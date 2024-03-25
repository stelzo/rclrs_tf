use geometry_msgs::msg::TransformStamped;
use tf2_msgs::msg::TFMessage;

use geometry_msgs::msg::TransformStamped;
use tf2_msgs::msg::TFMessage;

use crate::tf_error::TfError;

pub struct TfBroadcaster {
    publisher: rclrs::Publisher<TFMessage>,
}

impl TfBroadcaster {
    /// Create a new TfBroadcaster
    #[track_caller]
    pub fn new(node: &mut rclrs::Node) -> Self {
        Self {
            publisher: node.create_publisher("/tf", rclrs::QOS_PROFILE_DEFAULT).unwrap(),
        }
    }

    /// Broadcast transform
    pub fn send_transform(&self, tf: TransformStamped) -> Result<(), TfError> {
        let tf_message = TFMessage {
            transforms: vec![tf],
        };
        // TODO: handle error correctly
        self.publisher
            .publish(&tf_message)
            .map_err(|err| TfError::Rclrs(err.to_string()))
    }
}
