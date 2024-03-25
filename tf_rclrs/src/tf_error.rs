use std::collections::{HashMap, HashSet};

use builtin_interfaces::msg::Time;
use geometry_msgs::msg::TransformStamped;
use thiserror::Error;

/// Enumerates the different types of errors
#[derive(Clone, Debug, Error)]
#[non_exhaustive]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    #[error("tf_rclrs: AttemptedLookupInPast {:?} < {:?}",.0, .1)]
    AttemptedLookupInPast(Time, Box<TransformStamped>),
    /// Error due to the transform not yet being available.
    #[error("tf_rclrs: AttemptedLookupInFuture {:?} < {:?}",.0, .1)]
    AttemptedLookUpInFuture(Box<TransformStamped>, Time),
    /// There is no path between the from and to frame.
    #[error("tf_rclrs: CouldNotFindTransform {} -> {} ({:?})", .0, .1, .2)]
    CouldNotFindTransform(String, String, HashMap<String, HashSet<String>>),
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    #[error("tf_rclrs: CouldNotAcquireLock")]
    CouldNotAcquireLock,
    /// Error of rclrs
    #[error("tf_rclrs: rclrs error {:?}", .0)]
    Rclrs(String),
}
