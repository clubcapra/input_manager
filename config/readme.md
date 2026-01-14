This directory is used for configs. They are unfortunately stored in YAML, which is objectively inferior to JSON, however it allows comments to clarify inputs to allow a wider range of device types in the future.

**READONLY_example_config.yaml shall NOT BE MODIFIED or used at runtime**. It is an example with comments explaining how a config file should be made to support a device.

default_config.yaml is the default, basic config used by CAPRA at any point in time. It is not persistent and therefore should not be relied upon blindly.

other configs will be gitignored.

pr test
