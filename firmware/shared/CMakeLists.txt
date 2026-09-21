# Header only: the wire contract both ends include, see link_protocol_defs.h.
# Nothing to compile, the ESP-IDF side only needs it on the include path. The
# Keil side reaches the very same file through its own IncludePath entry.
idf_component_register(INCLUDE_DIRS ".")
