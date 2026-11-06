# Losslessly encoded UTF-8 text for the COC -> TIC ingress.
# radio_signal packs five base-3 trits into each uint8 (five trits fit in 0..242).
# Ternary digits map to agreed signal states: 0 -> -1, 1 -> 0, 2 -> +1.
# Six trits encode each UTF-8 byte; symbol_count excludes final packing padding.
uint8[] radio_signal
uint32 symbol_count

# Unique ID for this transmission; useful for tracing and future response correlation.
string radio_id

# Logical recipient selector only. This is not authentication or encryption.
string radio_channel
