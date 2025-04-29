# SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0
import hashlib
import logging
from typing import Callable

import pytest
from pytest_embedded import Dut
from pytest_embedded_qemu.app import QemuApp
from pytest_embedded_qemu.dut import QemuDut
from pytest_embedded_idf.dut import IdfDut


@pytest.mark.supported_targets
@pytest.mark.esp32
def test_greenhouse_functionality(
    dut: IdfDut, log_minimum_free_heap_size: Callable[..., None]
) -> None:
    # Check for system initialization message
    dut.expect('Initializing system...')
    
    # Check for WiFi initialization
    dut.expect('wifi_init_sta finished')
    
    # Check for sensors initialization
    dut.expect('Initializing sensors')
    
    # Check for relays initialization
    dut.expect('Initializing relays')
    
    # Verify ventilation and cooling systems are initialized
    dut.expect('Initializing ventilation system')
    dut.expect('Initializing cooling system')
    
    # Check for MQTT initialization
    dut.expect('MQTT initialized')
    
    # Log heap size
    log_minimum_free_heap_size()


@pytest.mark.linux
@pytest.mark.host_test
def test_hello_world_linux(dut: IdfDut) -> None:
    dut.expect('Hello world!')


@pytest.mark.linux
@pytest.mark.host_test
@pytest.mark.macos_shell
def test_hello_world_macos(dut: Dut) -> None:
    dut.expect('Hello world!')


def verify_elf_sha256_embedding(app: QemuApp, sha256_reported: str) -> None:
    sha256 = hashlib.sha256()
    with open(app.elf_file, 'rb') as f:
        sha256.update(f.read())
    sha256_expected = sha256.hexdigest()

    logging.info(f'ELF file SHA256: {sha256_expected}')
    logging.info(f'ELF file SHA256 (reported by the app): {sha256_reported}')

    # the app reports only the first several hex characters of the SHA256, check that they match
    if not sha256_expected.startswith(sha256_reported):
        raise ValueError('ELF file SHA256 mismatch')


@pytest.mark.esp32  # we only support qemu on esp32 for now
@pytest.mark.host_test
@pytest.mark.qemu
def test_hello_world_host(app: QemuApp, dut: QemuDut) -> None:
    sha256_reported = (
        dut.expect(r'ELF file SHA256:\s+([a-f0-9]+)').group(1).decode('utf-8')
    )
    verify_elf_sha256_embedding(app, sha256_reported)

    dut.expect('Hello world!')


@pytest.mark.qemu
@pytest.mark.qemu_xtensa
def test_hello_world_qemu_xtensa(app: QemuApp, dut: QemuDut) -> None:
    dut.expect('Hello world!')
