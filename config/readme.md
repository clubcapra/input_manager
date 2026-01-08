This directory is used for configs. They are unfortunately stored in YAML, which is objectively inferior to JSON, however it allows comments to clarify inputs to allow a wider range of device types in the future.

READONLY_example_config.yaml is an example with commments explaining how a config file should be made to support a device. It shall not me used or modified for runtime use.

default_config.yaml is the default, basic config used by CAPRA at any point in time. It is not persistent and therefore should not be relied upon blindly.

other configs will be gitignored.