//! A tiny, generic "run this off the UI thread, poll for the result
//! once per frame" utility -- see the "Background heavy computations"
//! plan (development log) for the full motivation: `alladin-pcb` is a
//! single-threaded `egui`/`eframe` app, so any expensive, synchronous
//! call inside [`crate::app::PcbApp::ui`] (a zone flood-fill, an A*
//! routing search, a whole `run_batch`, a manufacturing export) stops
//! the window from repainting or handling *any* input for however long
//! that call takes -- long enough, on a busy board, to trip the OS's
//! "application not responding" watchdog.
//!
//! [`BackgroundJob`] is deliberately the *only* new mechanism this
//! introduces. It's the exact same "spawn a `std::thread`, hand the
//! result back over an `mpsc::Receiver`, poll it non-blockingly once
//! per frame" shape `crate::external_router::run_autoroute` and
//! `crate::lcsc::fetch_in_background` already use for their own
//! background work -- generalized here into one small, independently
//! testable type instead of being copy-pasted again for every new
//! call site `crate::app` needs it for (zone fill, routing search,
//! manufacturing export, net-continuity check, batch operations).
//!
//! What this module does *not* do: it never touches `BoardDoc`/
//! `EditorState`/`Screen` itself, and it has no opinion on what `T` is
//! -- every actual "clone the board, compute something pure, merge the
//! result back on the UI thread" policy lives in `crate::app`, right
//! next to the `*_write`/`*_json` functions it wraps.

use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;

/// One computation running on its own OS thread. `T` is whatever the
/// closure handed to [`Self::spawn`] returns -- in practice, either a
/// finished JSON [`serde_json::Value`] (for a purely read-only job) or
/// a closure that still needs to run against the *live* `Screen` to
/// merge the result in (see `crate::app`'s own `JobResult` alias) --
/// [`BackgroundJob`] itself doesn't care which.
pub struct BackgroundJob<T> {
    rx: Receiver<T>,
    /// Set once [`Self::poll`] has returned anything other than
    /// [`JobPoll::Pending`] -- polling again past that point would
    /// otherwise re-observe a disconnected channel and keep reporting
    /// [`JobPoll::Lost`] forever instead of simply never firing again,
    /// which would be surprising for a caller that (incorrectly) polls
    /// a job it should have already discarded.
    finished: bool,
}

/// [`BackgroundJob::poll`]'s result -- three states, not a plain
/// `Option<T>`, so a worker thread that panicked (a genuine bug, but
/// one a single bad board/geometry input should never be able to
/// freeze the whole UI over) is distinguishable from "still running":
/// a caller can clear whatever "busy" UI state it set and surface a
/// real error instead of waiting forever on a channel that will now
/// never receive anything.
pub enum JobPoll<T> {
    Pending,
    Ready(T),
    /// The worker thread's [`Sender`](mpsc::Sender) was dropped without
    /// ever sending a value -- only possible if that thread panicked
    /// (every [`Self::spawn`] closure runs to completion and sends
    /// exactly once otherwise).
    Lost,
}

impl<T: Send + 'static> BackgroundJob<T> {
    /// Spawns `f` on a brand-new OS thread and returns immediately --
    /// never blocks. `f` must own everything it touches (typically a
    /// cheap `Clone` of a `Node`/`BoardDoc`/outline taken right before
    /// this call, per this module's own doc comment): the whole point
    /// is that the UI thread keeps repainting/handling input while `f`
    /// runs, so it must not need anything back from the UI thread to
    /// finish.
    pub fn spawn(f: impl FnOnce() -> T + Send + 'static) -> Self {
        let (tx, rx) = mpsc::channel();
        thread::Builder::new()
            .name("alladin-pcb-bg".to_string())
            .spawn(move || {
                let _ = tx.send(f());
            })
            .expect("spawning a background worker thread must succeed");
        Self { rx, finished: false }
    }

    /// A job that's already finished, without ever spawning a thread --
    /// for call sites that discover up front (before any real work is
    /// needed, e.g. "no board is open yet") that there's nothing to
    /// background at all, but still want to hand back the same
    /// [`BackgroundJob`] type every other call site does, so
    /// `crate::app`'s dispatch stays uniform. The very next [`Self::poll`]
    /// call returns [`JobPoll::Ready`].
    pub fn ready(value: T) -> Self {
        let (tx, rx) = mpsc::channel();
        let _ = tx.send(value);
        Self { rx, finished: false }
    }

    /// Non-blocking check for a result -- called once per frame from
    /// [`crate::app::PcbApp::ui`], the same `try_recv()` shape that
    /// method already uses for `mcp_rx`/`lcsc_fetch`/the autoroute
    /// event channel.
    pub fn poll(&mut self) -> JobPoll<T> {
        if self.finished {
            return JobPoll::Lost;
        }
        match self.rx.try_recv() {
            Ok(value) => {
                self.finished = true;
                JobPoll::Ready(value)
            }
            Err(TryRecvError::Empty) => JobPoll::Pending,
            Err(TryRecvError::Disconnected) => {
                self.finished = true;
                JobPoll::Lost
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn poll_reports_pending_then_ready_once_the_worker_thread_finishes() {
        let mut job = BackgroundJob::spawn(|| {
            thread::sleep(Duration::from_millis(50));
            42
        });
        assert!(matches!(job.poll(), JobPoll::Pending), "must not block waiting for the worker");

        let value = loop {
            match job.poll() {
                JobPoll::Pending => thread::sleep(Duration::from_millis(10)),
                JobPoll::Ready(v) => break v,
                JobPoll::Lost => panic!("a normally-returning closure must never report Lost"),
            }
        };
        assert_eq!(value, 42);
    }

    #[test]
    fn poll_after_ready_never_panics_and_reports_lost_not_ready_again() {
        let mut job = BackgroundJob::ready("done".to_string());
        assert!(matches!(job.poll(), JobPoll::Ready(v) if v == "done"));
        // A caller that (incorrectly) polls again must get a clear
        // "nothing more is coming", not a panic or a stale repeat.
        assert!(matches!(job.poll(), JobPoll::Lost));
    }

    #[test]
    fn a_panicking_worker_is_reported_as_lost_not_left_pending_forever() {
        let mut job: BackgroundJob<u32> = BackgroundJob::spawn(|| panic!("boom"));
        let result = loop {
            match job.poll() {
                JobPoll::Pending => thread::sleep(Duration::from_millis(10)),
                other => break other,
            }
        };
        assert!(matches!(result, JobPoll::Lost));
    }

    #[test]
    fn ready_never_spawns_a_thread_and_resolves_on_the_very_first_poll() {
        let mut job = BackgroundJob::ready(7);
        assert!(matches!(job.poll(), JobPoll::Ready(7)));
    }
}
