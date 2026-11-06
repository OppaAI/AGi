"""Round-trip and malformed-input tests for the Web Teleops Console radio-inspired codec."""

import pytest

from tms.radio_encoding import decode_text, encode_text


@pytest.mark.parametrize(
    "text",
    [
        "Hello world!",
        "/web Hello, Grace!",
        "I'm aiko—こんにちは 👋",
        "",
    ],
)
def test_encode_decode_preserves_text(text: str) -> None:
    packed, symbol_count = encode_text(text)
    assert decode_text(packed, symbol_count) == text


def test_codec_packs_five_trits_per_byte() -> None:
    packed, symbol_count = encode_text("Hello world!")
    assert symbol_count == len("Hello world!".encode("utf-8")) * 6
    assert len(packed) == (symbol_count + 4) // 5


@pytest.mark.parametrize(
    ("packed", "symbol_count"),
    [
        (b"\xff\x00", 6),  # packed values above 242 cannot be five base-3 trits
        (b"\x00", 5),  # six-trit UTF-8 byte framing is required
        (b"\x00", 6),  # one packed byte cannot hold six trits
        (b"\x00\x01", 6),  # trailing padding must be zero
    ],
)
def test_decoder_rejects_invalid_payload(packed: bytes, symbol_count: int) -> None:
    with pytest.raises(ValueError):
        decode_text(packed, symbol_count)


def test_decoder_rejects_non_utf8_byte_sequences() -> None:
    packed, symbol_count = encode_text("A")
    # Replace the six trits with the base-3 form of byte 0xFF.
    digits = [0, 0, 0, 0, 0, 0]
    value = 0xFF
    for index in range(5, -1, -1):
        value, digits[index] = divmod(value, 3)
    trits = digits + [0, 0, 0, 0]
    encoded = bytearray()
    for start in range(0, 10, 5):
        group = trits[start : start + 5]
        packed_value = 0
        for digit in group:
            packed_value = packed_value * 3 + digit
        encoded.append(packed_value)
    assert symbol_count == 6
    with pytest.raises(ValueError, match="valid UTF-8"):
        decode_text(encoded[:2], 6)
