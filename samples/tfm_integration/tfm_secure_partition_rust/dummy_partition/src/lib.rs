// Copyright (c) 2021 Nordic Semiconductor ASA
// Copyright (c) 2025 Dmitrii Sharshakov
// SPDX-License-Identifier: Apache-2.0

#![no_std]

mod bindings;

use bindings::*;
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    unsafe {
        psa_panic();
    }

    unreachable!()
}

static SECRETS: [[u8; 16]; 5] = [
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    [1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    [2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    [3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    [4, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
];

// TODO cleanup, refactor
fn tfm_dp_secret_digest_ipc(msg: &mut PsaMsg) -> i32 {
    if msg.in_size[0] != core::mem::size_of::<u32>() {
        return PSA_ERROR_PROGRAMMER_ERROR;
    }

    let mut secret_index_bytes = [0u8; 4];
    let num = unsafe {
        psa_read(
            msg.handle,
            0,
            secret_index_bytes.as_mut_ptr(),
            secret_index_bytes.len(),
        )
    };
    if num != msg.in_size[0] {
        return PSA_ERROR_PROGRAMMER_ERROR;
    }

    let secret_index = u32::from_le_bytes(secret_index_bytes) as usize;
    if secret_index >= SECRETS.len() {
        return psa_crypto::ffi::PSA_ERROR_INVALID_ARGUMENT;
    }

    let mut out_size = msg.out_size[0];
    let mut digest = [0u8; 32];
    if out_size != digest.len() {
        return psa_crypto::ffi::PSA_ERROR_INVALID_ARGUMENT;
    }

    psa_crypto::init().unwrap();

    let secret = &SECRETS[secret_index];
    out_size = psa_crypto::operations::hash::hash_compute(
        psa_crypto::types::algorithm::Hash::Sha256,
        secret,
        &mut digest,
    )
    .unwrap();

    if digest.len() != out_size {
        return PSA_ERROR_PROGRAMMER_ERROR;
    }

    unsafe {
        psa_write(msg.handle, 0, digest.as_ptr(), out_size);
    }

    psa_crypto::ffi::PSA_SUCCESS
}

fn dp_signal_handle(signal: u32) {
    let mut msg: PsaMsg = PsaMsg::default();

    let mut status = unsafe { psa_get(signal, &mut msg) };
    if status != psa_crypto::ffi::PSA_SUCCESS {
        panic!();
    }

    match msg.msg_type {
        PSA_IPC_CONNECT => {
            unsafe { psa_reply(msg.handle, psa_crypto::ffi::PSA_SUCCESS) };
        }
        PSA_IPC_CALL => {
            status = tfm_dp_secret_digest_ipc(&mut msg);
            unsafe { psa_reply(msg.handle, status) };
        }
        PSA_IPC_DISCONNECT => {
            unsafe { psa_reply(msg.handle, psa_crypto::ffi::PSA_SUCCESS) };
        }
        _ => panic!(),
    }
}

#[no_mangle]
extern "C" fn tfm_dp_req_mngr_init() -> i32 {
    loop {
        let signals: u32 = unsafe { psa_wait(PSA_WAIT_ANY, PSA_BLOCK) };
        if signals & TFM_DP_SECRET_DIGEST_SIGNAL == TFM_DP_SECRET_DIGEST_SIGNAL {
            dp_signal_handle(TFM_DP_SECRET_DIGEST_SIGNAL);
        } else {
            panic!();
        }
    }
}
