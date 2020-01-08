;;; Directory Local Variables
;;; For more information see (info "(emacs) Directory Variables")

((c++-mode
  (flycheck-gcc-language-standard . "c++17")
  (eval progn
        (let
            ((includes-list
              (list
               (substitute-in-file-name "$rcsw/include")
               (substitute-in-file-name "$rcppsw/include")
               (substitute-in-file-name "$fordyca/include")
               (substitute-in-file-name "$cosm/include")
               (concat
                (projectile-project-root)
                "include")
               "/usr/include/lua5.2" "/usr/include/x86_64-linux-gnu/qt5/"
               "/usr/include/x86_64-linux-gnu/qt5/QtWidgets"
               "/usr/include/x86_64-linux-gnu/qt5/QtGui"
               "/usr/include/x86_64-linux-gnu/qt5/QtCore"
               "/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++-64")))
          (setq flycheck-clang-include-path includes-list)
          (setq flycheck-gcc-include-path includes-list)
          (add-to-list 'flycheck-clang-args "-std=c++17")
          (add-to-list 'flycheck-clang-args "-fPIC")
          (add-to-list 'flycheck-clang-args (concat "-isystem" (substitute-in-file-name
                                                                 "$rcppsw")))
          (add-to-list 'flycheck-clang-args (concat "-isystem" (substitute-in-file-name
                                                                "$localroot/include")))
          (add-to-list 'flycheck-clang-args (concat "-isystem" "/usr/include/eigen3"))
          (add-to-list 'flycheck-clang-definitions "COSM_HAL_TARGET=HAL_TARGET_ARGOS_FOOTBOT")

          (add-to-list 'flycheck-gcc-args "-fPIC")
          (add-to-list 'flycheck-gcc-definitions "COSM_HAL_TARGET=HAL_TARGET_ARGOS_FOOTBOT")
          (add-to-list 'flycheck-gcc-args "-std=c++17")
          (add-to-list 'flycheck-gcc-args (concat "-isystem" (substitute-in-file-name
                                                              "$rcppsw")))
          (add-to-list 'flycheck-gcc-args (concat "-isystem" (substitute-in-file-name
                                                              "$localroot/include")))
          (add-to-list 'flycheck-gcc-args (concat "-isystem" "/usr/include/eigen3"))
          )
        (c++-mode))))


;;; end of .dir-locals.el
