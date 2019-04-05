
(cl:in-package :asdf)

(defsystem "apclab_dset_collector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ExecutionStatus" :depends-on ("_package_ExecutionStatus"))
    (:file "_package_ExecutionStatus" :depends-on ("_package"))
  ))