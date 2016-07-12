
(cl:in-package :asdf)

(defsystem "map_maker-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MapTalk" :depends-on ("_package_MapTalk"))
    (:file "_package_MapTalk" :depends-on ("_package"))
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
  ))