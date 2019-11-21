# Change Log
All notable changes to this project will be documented in this file. This change log follows the conventions of [keepachangelog.com](http://keepachangelog.com/).

## [Unreleased]
### Changed
- Speed up decode of 0 stripped MAVLink 2 packets.
- Fields of a parsed message are no longer retained by default. To retain them pass the `:retain-fields? true` flag when parsing sources.

## [0.1.1] - 2019-02-11
### Changeed
- Removed unneeded reflection.
- Speed up array decoding, reduce memory consumed for parsing ranges.
- Speed up computing checksums

## 0.1.0 - 2018-04-02
### Added
- Initial release

[Unreleased]: https://github.com/wicked-shell/clj-mavlink/compare/0.1.1...HEAD
[0.1.1]: https://github.com/wickedshell/clj-mavlink/compare/0.1.0...0.1.1
