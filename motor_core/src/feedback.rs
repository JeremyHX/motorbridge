use crate::error::{MotorError, Result};
use std::sync::Mutex;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct FeedbackSnapshot<T> {
    pub state: T,
    pub received_at: Instant,
    pub sequence: u64,
}

impl<T> FeedbackSnapshot<T> {
    pub fn age(&self) -> Duration {
        self.received_at.elapsed()
    }
}

#[derive(Debug)]
pub struct FeedbackCache<T> {
    inner: Mutex<FeedbackCacheInner<T>>,
}

#[derive(Debug)]
struct FeedbackCacheInner<T> {
    latest: Option<FeedbackSnapshot<T>>,
    sequence: u64,
}

impl<T> FeedbackCache<T> {
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(FeedbackCacheInner {
                latest: None,
                sequence: 0,
            }),
        }
    }

    pub fn update(&self, state: T) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("feedback cache lock poisoned".to_string()))?;
        inner.sequence = inner.sequence.saturating_add(1);
        inner.latest = Some(FeedbackSnapshot {
            state,
            received_at: Instant::now(),
            sequence: inner.sequence,
        });
        Ok(())
    }

    pub fn latest(&self) -> Result<Option<FeedbackSnapshot<T>>>
    where
        T: Clone,
    {
        Ok(self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("feedback cache lock poisoned".to_string()))?
            .latest
            .clone())
    }

    pub fn sequence(&self) -> Result<u64> {
        Ok(self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("feedback cache lock poisoned".to_string()))?
            .sequence)
    }
}

impl<T> Default for FeedbackCache<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_tracks_age_and_sequence() {
        let cache = FeedbackCache::new();
        assert!(cache.latest().expect("empty cache").is_none());
        assert_eq!(cache.sequence().expect("empty sequence"), 0);

        cache.update(10).expect("first update");
        let first = cache.latest().expect("first snapshot").expect("state");
        std::thread::sleep(Duration::from_millis(2));
        assert!(first.age() >= Duration::from_millis(1));
        assert_eq!(first.state, 10);
        assert_eq!(first.sequence, 1);

        cache.update(20).expect("second update");
        let second = cache.latest().expect("second snapshot").expect("state");
        assert_eq!(second.state, 20);
        assert_eq!(second.sequence, 2);
        assert!(second.received_at > first.received_at);
        assert!(second.age() < first.age());
    }
}
