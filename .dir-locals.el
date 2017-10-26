;;; .dir-locals.el --

;;; Commentary:

;;; Code:
((nil .
      ((eval  . (progn
                  (require 'rtags-init)
                  (require 'irony-mode-init)
                  (let ((includes-list (list
                                        (substitute-in-file-name "$rcsw/include")
                                        (substitute-in-file-name "$rcppsw/include")
                                        (concat (projectile-project-root) "include")
                                        "/usr/include/lua5.2"
                                        "/usr/include/x86_64-linux-gnu/qt5/"
                                        "/usr/include/x86_64-linux-gnu/qt5/QtWidgets"
                                        "/usr/include/x86_64-linux-gnu/qt5/QtGui"
                                        "/usr/include/x86_64-linux-gnu/qt5/QtCore"
                                        "/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++-64"
                                        )))
                    (setq flycheck-clang-include-path includes-list)
                    (setq flycheck-clang-language-standard  "c++1l")
                    (add-to-list 'flycheck-clang-args "-fPIC")
                    (setq flycheck-gcc-include-path includes-list)
                    (setq flycheck-gcc-language-standard  "c++11")
                    (add-to-list 'flycheck-gcc-args "-fPIC")

                    (setq compile-command (concat "make -C" (concat (projectile-project-root) "build")))
                    (add-hook 'c++-mode-hook 'google-style-hook)
                    )
                  (let ((cc-search-dirs (list (concat (projectile-project-root) "src/*")
                                              (concat (projectile-project-root) "include/fordyca/*")
                                              (concat (projectile-project-root) "include/fordyca")
                                              (concat (projectile-project-root) "include"))
                                        ))
                    (setq cc-search-directories cc-search-dirs))
                  (c++-mode)
              )
         ))
      ))

;;; end of .dir-locals.el
