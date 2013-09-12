# Package which contains this spec file
string package

# Type of spec file, one of:
#   'capability_interface'
#   'semantic_capability_interface'
#   'capability_provider'
string type

# Raw string content of the spec file
string content

# The name of the default provider (pulled from ROS param's)
# (only used for capability_interface specs)
string default_provider
