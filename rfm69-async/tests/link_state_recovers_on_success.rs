// SPDX-License-Identifier: AGPL-3.0-only

mod common;

use common::{pace_time, run_test};
use futures::FutureExt;
use rfm69_async::{LinkState, TrxError};

#[test]
fn link_flips_back_up_on_recover_success() {
    run_test(async |stack, trx| {
        // Drive the link Down with a streak of recv errors.
        for _ in 0..3 {
            trx.inject_err(TrxError::Spi);
        }
        {
            let wait = stack.wait_link_down().fuse();
            let pacer = pace_time(50, 1).fuse();
            futures::pin_mut!(wait);
            futures::pin_mut!(pacer);
            futures::select! {
                _ = wait => {}
                _ = pacer => panic!("wait_link_down never resolved"),
            }
        }
        assert!(matches!(stack.link_state(), LinkState::Down));

        // Once Down, the Runner is stuck in the recovery loop — random
        // `recv` successes wouldn't be observable (recv isn't called until
        // recovery clears the link). Queue a successful recover instead.
        trx.inject_recover_ok();
        {
            let wait = stack.wait_link_up().fuse();
            // Long enough for one recover_backoff (500 ms) to elapse and the
            // next recovery attempt to be polled.
            let pacer = pace_time(700, 1).fuse();
            futures::pin_mut!(wait);
            futures::pin_mut!(pacer);
            futures::select! {
                _ = wait => {}
                _ = pacer => panic!("wait_link_up never resolved"),
            }
        }
        assert!(matches!(stack.link_state(), LinkState::Up));
        assert!(trx.recover_calls() >= 1);
    });
}
