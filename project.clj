(defproject clj-mavlink "0.1.0"
  :description "Clojure MAVLink"
  :url "https://github.com/WickedShell/clj-mavlink"
  :license {:name "Eclipse Public License"
            :url "http://www.eclipse.org/legal/epl-v10.html"}
  :dependencies [[org.clojure/clojure "1.8.0"]
                 [org.clojure/data.xml "0.0.8"]
                 [org.clojure/data.zip "0.1.2"]
                 ]
  :profiles {:dev {:dependencies [ [criterium "0.4.4"] ]
                  :java-source-paths ["test/mavlink"
                                      "src/java"
                                      ]
                  }
            }
  :global-vars {*warn-on-reflection* true})
