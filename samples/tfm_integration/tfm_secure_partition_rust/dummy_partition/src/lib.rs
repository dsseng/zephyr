// Copyright (c) 2021 Nordic Semiconductor ASA
// Copyright (c) 2025 Dmitrii Sharshakov
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

use core::panic::PanicInfo;
mod bindings;
use crate::bindings::*;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    unsafe {
        psa_panic();
    }

    unreachable!()
}

const NUM_SECRETS: usize = 5;

struct DpSecret {
    secret: [u8; 16],
}

static SECRETS: [DpSecret; NUM_SECRETS] = [
    DpSecret {
        secret: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    },
    DpSecret {
        secret: [1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    },
    DpSecret {
        secret: [2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    },
    DpSecret {
        secret: [3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    },
    DpSecret {
        secret: [4, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    },
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

    let mut out_size = msg.out_size[0];

    if secret_index >= NUM_SECRETS {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    let mut digest = [0u8; 32];
    if out_size != digest.len() {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    let secret = &SECRETS[secret_index].secret;

    unsafe {
        psa_hash_compute(
            PSA_ALG_SHA_256,
            secret.as_ptr(),
            secret.len(),
            digest.as_mut_ptr(),
            digest.len(),
            &mut out_size,
        );
    }

    if digest.len() != out_size {
        return PSA_ERROR_PROGRAMMER_ERROR;
    }

    unsafe {
        psa_write(msg.handle, 0, digest.as_ptr(), out_size);
    }

    PSA_SUCCESS
}

fn dp_signal_handle(signal: u32) {
    let mut msg: PsaMsg = PsaMsg::default();

    let mut status = unsafe { psa_get(signal, &mut msg) };
    if status != PSA_SUCCESS {
        panic!();
    }

    match msg.msg_type {
        PSA_IPC_CONNECT => {
            unsafe { psa_reply(msg.handle, PSA_SUCCESS) };
        }
        PSA_IPC_CALL => {
            status = tfm_dp_secret_digest_ipc(&mut msg);
            unsafe { psa_reply(msg.handle, status) };
        }
        PSA_IPC_DISCONNECT => {
            unsafe { psa_reply(msg.handle, PSA_SUCCESS) };
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
