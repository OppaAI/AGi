"""Losslessly pack UTF-8 text into a compact ternary radio-inspired signal.

Each UTF-8 byte is represented by six base-3 symbols. The symbols correspond
to the three agreed signal states (-1, 0, +1), mapped to base-3 digits
(0, 1, 2). Five trits are packed into each output byte. This is a digital
encoding for the ROS payload; it does not generate a physical radio waveform.
"""

from __future__ import annotations

from collections.abc import Iterable

TRITS_PER_BYTE = 6
TRITS_PER_PACKED_BYTE = 5
BASE = 3
MAX_PACKED_VALUE = BASE**TRITS_PER_PACKED_BYTE - 1


def encode_text(text: str) -> tuple[bytes, int]:
    """Encode arbitrary Unicode text as packed ternary symbols.

    Returns the packed bytes and exact trit count. Six trits per UTF-8 byte
    make decoding unambiguous without delimiters; the count excludes padding.
    """
    if not isinstance(text, str):
        raise TypeError("text must be a string")

    trits: list[int] = []
    for byte in text.encode("utf-8"):
        digits = [0] * TRITS_PER_BYTE
        value = byte
        for index in range(TRITS_PER_BYTE - 1, -1, -1):
            value, digits[index] = divmod(value, BASE)
        if value:
            raise ValueError("UTF-8 byte does not fit the six-trit encoding")
        trits.extend(digits)

    packed = bytearray()
    for start in range(0, len(trits), TRITS_PER_PACKED_BYTE):
        group = trits[start : start + TRITS_PER_PACKED_BYTE]
        group.extend([0] * (TRITS_PER_PACKED_BYTE - len(group)))
        value = 0
        for digit in group:
            value = value * BASE + digit
        packed.append(value)

    return bytes(packed), len(trits)


def decode_text(packed_signal: Iterable[int], symbol_count: int) -> str:
    """Decode packed ternary symbols into their original Unicode text."""
    if not isinstance(symbol_count, int) or isinstance(symbol_count, bool):
        raise TypeError("symbol_count must be an integer")
    if symbol_count < 0 or symbol_count % TRITS_PER_BYTE:
        raise ValueError("symbol_count must be a non-negative multiple of six")

    packed = bytes(packed_signal)
    expected_length = (symbol_count + TRITS_PER_PACKED_BYTE - 1) // TRITS_PER_PACKED_BYTE
    if len(packed) != expected_length:
        raise ValueError("packed signal length does not match symbol_count")
    if any(value > MAX_PACKED_VALUE for value in packed):
        raise ValueError("packed signal contains an invalid base-3 group")

    trits: list[int] = []
    for value in packed:
        group = [0] * TRITS_PER_PACKED_BYTE
        for index in range(TRITS_PER_PACKED_BYTE - 1, -1, -1):
            value, group[index] = divmod(value, BASE)
        trits.extend(group)

    if any(trits[symbol_count:]):
        raise ValueError("packed signal has non-zero trailing padding")
    trits = trits[:symbol_count]

    decoded = bytearray()
    for start in range(0, len(trits), TRITS_PER_BYTE):
        value = 0
        for digit in trits[start : start + TRITS_PER_BYTE]:
            value = value * BASE + digit
        if value > 0xFF:
            raise ValueError("ternary group is outside the UTF-8 byte range")
        decoded.append(value)

    try:
        return decoded.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("radio signal does not contain valid UTF-8") from error
