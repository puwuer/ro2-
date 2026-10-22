#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to mycar_interfaces__msg__SystemStatus
/// 系统工作模式常量

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SystemStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// 当前工作模式: PATROL / TRACK / SEARCH
    pub mode: u8,

    /// 是否检测到跟踪目标(人)
    pub target_visible: bool,

    /// 目标在 map 坐标系下的位置, target_visible=false 时忽略
    pub target_pose: geometry_msgs::msg::PoseStamped,

    /// 检测置信度 0~1
    pub confidence: f32,

    /// 当前(或最近)巡检点索引, 非巡检模式时保留最后一次的值
    pub waypoint_index: i32,

    /// 最近一次状态切换原因/说明, 仅用于调试显示
    pub detail: std::string::String,

}

impl SystemStatus {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PATROL: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TRACK: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SEARCH: u8 = 2;

}


impl Default for SystemStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SystemStatus::default())
  }
}

impl rosidl_runtime_rs::Message for SystemStatus {
  type RmwMsg = super::msg::rmw::SystemStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        mode: msg.mode,
        target_visible: msg.target_visible,
        target_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.target_pose)).into_owned(),
        confidence: msg.confidence,
        waypoint_index: msg.waypoint_index,
        detail: msg.detail.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      mode: msg.mode,
      target_visible: msg.target_visible,
        target_pose: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.target_pose)).into_owned(),
      confidence: msg.confidence,
      waypoint_index: msg.waypoint_index,
        detail: msg.detail.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      mode: msg.mode,
      target_visible: msg.target_visible,
      target_pose: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.target_pose),
      confidence: msg.confidence,
      waypoint_index: msg.waypoint_index,
      detail: msg.detail.to_string(),
    }
  }
}


