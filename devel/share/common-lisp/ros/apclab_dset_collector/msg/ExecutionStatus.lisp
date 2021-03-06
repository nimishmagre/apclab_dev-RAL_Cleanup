; Auto-generated. Do not edit!


(cl:in-package apclab_dset_collector-msg)


;//! \htmlinclude ExecutionStatus.msg.html

(cl:defclass <ExecutionStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass ExecutionStatus (<ExecutionStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecutionStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecutionStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apclab_dset_collector-msg:<ExecutionStatus> is deprecated: use apclab_dset_collector-msg:ExecutionStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ExecutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apclab_dset_collector-msg:header-val is deprecated.  Use apclab_dset_collector-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ExecutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apclab_dset_collector-msg:status-val is deprecated.  Use apclab_dset_collector-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecutionStatus>) ostream)
  "Serializes a message object of type '<ExecutionStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecutionStatus>) istream)
  "Deserializes a message object of type '<ExecutionStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecutionStatus>)))
  "Returns string type for a message object of type '<ExecutionStatus>"
  "apclab_dset_collector/ExecutionStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecutionStatus)))
  "Returns string type for a message object of type 'ExecutionStatus"
  "apclab_dset_collector/ExecutionStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecutionStatus>)))
  "Returns md5sum for a message object of type '<ExecutionStatus>"
  "5af2dbd9f0f51a7e50dfafa69867ed29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecutionStatus)))
  "Returns md5sum for a message object of type 'ExecutionStatus"
  "5af2dbd9f0f51a7e50dfafa69867ed29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecutionStatus>)))
  "Returns full string definition for message of type '<ExecutionStatus>"
  (cl:format cl:nil "Header header~%string status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecutionStatus)))
  "Returns full string definition for message of type 'ExecutionStatus"
  (cl:format cl:nil "Header header~%string status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecutionStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecutionStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecutionStatus
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
))
