; Auto-generated. Do not edit!


(cl:in-package map_maker-srv)


;//! \htmlinclude MapTalk-request.msg.html

(cl:defclass <MapTalk-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MapTalk-request (<MapTalk-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapTalk-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapTalk-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name map_maker-srv:<MapTalk-request> is deprecated: use map_maker-srv:MapTalk-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapTalk-request>) ostream)
  "Serializes a message object of type '<MapTalk-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapTalk-request>) istream)
  "Deserializes a message object of type '<MapTalk-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapTalk-request>)))
  "Returns string type for a service object of type '<MapTalk-request>"
  "map_maker/MapTalkRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapTalk-request)))
  "Returns string type for a service object of type 'MapTalk-request"
  "map_maker/MapTalkRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapTalk-request>)))
  "Returns md5sum for a message object of type '<MapTalk-request>"
  "6fe7530d7a5e37ff441ff3304b9b549d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapTalk-request)))
  "Returns md5sum for a message object of type 'MapTalk-request"
  "6fe7530d7a5e37ff441ff3304b9b549d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapTalk-request>)))
  "Returns full string definition for message of type '<MapTalk-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapTalk-request)))
  "Returns full string definition for message of type 'MapTalk-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapTalk-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapTalk-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapTalk-request
))
;//! \htmlinclude MapTalk-response.msg.html

(cl:defclass <MapTalk-response> (roslisp-msg-protocol:ros-message)
  ((category_list
    :reader category_list
    :initarg :category_list
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (x_list
    :reader x_list
    :initarg :x_list
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (y_list
    :reader y_list
    :initarg :y_list
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (z_list
    :reader z_list
    :initarg :z_list
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (num_IDs
    :reader num_IDs
    :initarg :num_IDs
    :type cl:fixnum
    :initform 0)
   (adjacency_array
    :reader adjacency_array
    :initarg :adjacency_array
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass MapTalk-response (<MapTalk-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapTalk-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapTalk-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name map_maker-srv:<MapTalk-response> is deprecated: use map_maker-srv:MapTalk-response instead.")))

(cl:ensure-generic-function 'category_list-val :lambda-list '(m))
(cl:defmethod category_list-val ((m <MapTalk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_maker-srv:category_list-val is deprecated.  Use map_maker-srv:category_list instead.")
  (category_list m))

(cl:ensure-generic-function 'x_list-val :lambda-list '(m))
(cl:defmethod x_list-val ((m <MapTalk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_maker-srv:x_list-val is deprecated.  Use map_maker-srv:x_list instead.")
  (x_list m))

(cl:ensure-generic-function 'y_list-val :lambda-list '(m))
(cl:defmethod y_list-val ((m <MapTalk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_maker-srv:y_list-val is deprecated.  Use map_maker-srv:y_list instead.")
  (y_list m))

(cl:ensure-generic-function 'z_list-val :lambda-list '(m))
(cl:defmethod z_list-val ((m <MapTalk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_maker-srv:z_list-val is deprecated.  Use map_maker-srv:z_list instead.")
  (z_list m))

(cl:ensure-generic-function 'num_IDs-val :lambda-list '(m))
(cl:defmethod num_IDs-val ((m <MapTalk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_maker-srv:num_IDs-val is deprecated.  Use map_maker-srv:num_IDs instead.")
  (num_IDs m))

(cl:ensure-generic-function 'adjacency_array-val :lambda-list '(m))
(cl:defmethod adjacency_array-val ((m <MapTalk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader map_maker-srv:adjacency_array-val is deprecated.  Use map_maker-srv:adjacency_array instead.")
  (adjacency_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapTalk-response>) ostream)
  "Serializes a message object of type '<MapTalk-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'category_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'category_list))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'x_list))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'y_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'y_list))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'z_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'z_list))
  (cl:let* ((signed (cl:slot-value msg 'num_IDs)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'adjacency_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'adjacency_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapTalk-response>) istream)
  "Deserializes a message object of type '<MapTalk-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'category_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'category_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'y_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'y_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'z_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'z_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_IDs) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'adjacency_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'adjacency_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapTalk-response>)))
  "Returns string type for a service object of type '<MapTalk-response>"
  "map_maker/MapTalkResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapTalk-response)))
  "Returns string type for a service object of type 'MapTalk-response"
  "map_maker/MapTalkResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapTalk-response>)))
  "Returns md5sum for a message object of type '<MapTalk-response>"
  "6fe7530d7a5e37ff441ff3304b9b549d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapTalk-response)))
  "Returns md5sum for a message object of type 'MapTalk-response"
  "6fe7530d7a5e37ff441ff3304b9b549d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapTalk-response>)))
  "Returns full string definition for message of type '<MapTalk-response>"
  (cl:format cl:nil "uint8[] category_list~%uint16[] x_list~%uint16[] y_list~%uint16[] z_list~%int16 num_IDs~%uint16[] adjacency_array~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapTalk-response)))
  "Returns full string definition for message of type 'MapTalk-response"
  (cl:format cl:nil "uint8[] category_list~%uint16[] x_list~%uint16[] y_list~%uint16[] z_list~%int16 num_IDs~%uint16[] adjacency_array~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapTalk-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'category_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'y_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'z_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'adjacency_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapTalk-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapTalk-response
    (cl:cons ':category_list (category_list msg))
    (cl:cons ':x_list (x_list msg))
    (cl:cons ':y_list (y_list msg))
    (cl:cons ':z_list (z_list msg))
    (cl:cons ':num_IDs (num_IDs msg))
    (cl:cons ':adjacency_array (adjacency_array msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapTalk)))
  'MapTalk-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapTalk)))
  'MapTalk-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapTalk)))
  "Returns string type for a service object of type '<MapTalk>"
  "map_maker/MapTalk")