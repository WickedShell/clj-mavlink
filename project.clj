(defproject clj-mavlink "0.1.3-SNAPSHOT"
  :description "Clojure MAVLink"
  :url "https://github.com/WickedShell/clj-mavlink"
  :license {:name "Eclipse Public License"
            :url "http://www.eclipse.org/legal/epl-v10.html"}
  :dependencies [[org.clojure/clojure "1.10.3"]
                 [org.clojure/core.async "0.3.443"]
                 [org.clojure/data.zip "1.0.0"]
                 ]
  :profiles {:dev {:dependencies [ [criterium "0.4.4"] ]
                  :java-source-paths ["test/mavlink"
                                      "src/java"]
                  }
            }
  :plugins [[perforate "0.3.4"]]

  :global-vars {*warn-on-reflection* true})
