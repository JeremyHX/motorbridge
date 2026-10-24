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

    /// 就地修改最新快照的状态字段:**不推进序号、不刷新 received_at**。
    /// 供"补丁式"字段桥接使用(HT 2026-10-24:`26 0E` 寄存器回帧把
    /// 温度/错误码补进状态缓存,与锚定帧 `27 01` 合为一次状态计数,
    /// 计数/新鲜度语义完全由锚定帧定义)。无快照时返回 `Ok(false)`
    /// (首帧未到,补丁无处落,静默丢弃)。序号与时间戳保持锚定帧
    /// 的值,`request_fresh_state` 的新鲜度判定不受补丁影响。
    pub fn patch_latest(&self, f: impl FnOnce(&mut T)) -> Result<bool> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("feedback cache lock poisoned".to_string()))?;
        match inner.latest.as_mut() {
            Some(snapshot) => {
                f(&mut snapshot.state);
                Ok(true)
            }
            None => Ok(false),
        }
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

    /// 2026-10-24(HT 两帧状态计数):patch_latest 只改字段,
    /// 序号与 received_at 保持锚定帧的值;无快照时 Ok(false)。
    #[test]
    fn patch_latest_mutates_state_without_bumping_sequence() {
        let cache = FeedbackCache::new();
        cache.update(10).expect("anchor update");
        let before = cache.latest().expect("snapshot").expect("state");
        std::thread::sleep(Duration::from_millis(2));

        let patched = cache
            .patch_latest(|state| *state = 20)
            .expect("patch applied");
        assert!(patched);
        let after = cache.latest().expect("snapshot").expect("state");
        assert_eq!(after.state, 20, "字段应被就地修改");
        assert_eq!(after.sequence, before.sequence, "序号不推进");
        assert_eq!(after.received_at, before.received_at, "时间戳不刷新");

        // 锚定帧到达前:补丁无处落,返回 false 而非报错
        let empty = FeedbackCache::<u32>::new();
        assert!(!empty.patch_latest(|_| {}).expect("empty patch"));
    }
}
