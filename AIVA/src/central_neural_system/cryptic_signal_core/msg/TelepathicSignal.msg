# Losslessly encoded UTF-8 text for the Web Teleops Console -> TIC ingress.
# signal packs five base-3 trits into each uint8 (five trits fit in 0..242).
# Ternary digits map to agreed signal states: 0 -> -1, 1 -> 0, 2 -> +1.
# Six trits encode each UTF-8 byte; pulse counts valid trits, excluding padding.
string source
uint32 pulse
uint8[] signal
string channel
string scheme

# Unique ID for tracing and response correlation.
string identifier
