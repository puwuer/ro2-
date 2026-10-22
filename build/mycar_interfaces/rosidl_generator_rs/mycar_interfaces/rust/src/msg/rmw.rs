#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "mycar_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__mycar_interfaces__msg__SystemStatus() -> *const std::ffi::c_void;
}

#[link(name = "mycar_interfaces__rosidl_generator_c")]
extern "C" {
    fn mycar_interfaces__msg__SystemStatus__init(msg: *mut SystemStatus) -> bool;
    fn mycar_interfaces__msg__SystemStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SystemStatus>, size: usize) -> bool;
    fn mycar_interfaces__msg__SystemStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SystemStatus>);
    fn mycar_interfaces__msg__SystemStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SystemStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<SystemStatus>) -> bool;
}

// Corresponds to mycar_interfaces__msg__SystemStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 系统工作模式常量

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SystemStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// 当前工作模式: PATROL / TRACK / SEARCH
    pub mode: u8,

    /// 是否检测到跟踪目标(人)
    pub target_visible: bool,

    /// 目标在 map 坐标系下的位置, target_visible=false 时忽略
    pub target_pose: geometry_msgs::msg::rmw::PoseStamped,

    /// 检测置信度 0~1
    pub confidence: f32,

    /// 当前(或最近)巡检点索引, 非巡检模式时保留最后一次的值
    pub waypoint_index: i32,

    /// 最近一次状态切换原因/说明, 仅用于调试显示
    pub detail: rosidl_runtime_rs::String,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !mycar_interfaces__msg__SystemStatus__init(&mut msg as *mut _) {
        panic!("Call to mycar_interfaces__msg__SystemStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SystemStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mycar_interfaces__msg__SystemStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mycar_interfaces__msg__SystemStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { mycar_interfaces__msg__SystemStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SystemStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SystemStatus where Self: Sized {
  const TYPE_NAME: &'static str = "mycar_interfaces/msg/SystemStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__mycar_interfaces__msg__SystemStatus() }
  }
}


