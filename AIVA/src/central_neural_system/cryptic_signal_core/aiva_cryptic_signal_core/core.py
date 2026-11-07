"""AIVA's registry for reversible telepathic signal encodings."""
from __future__ import annotations

from collections.abc import Callable

TRITS_PER_BYTE = 6
TRITS_PER_PACKED_BYTE = 5
BASE = 3
MAX_PACKED_VALUE = BASE**TRITS_PER_PACKED_BYTE - 1


class CrypticSignalCore:
    """Select and run AIVA-approved signal encoding strategies.

    Add future schemes as named encoder/decoder pairs. Unknown scheme names
    fail closed so peers never silently decode a payload with the wrong method.
    """

    DEFAULT_SCHEME = "ternary_utf8_v1"

    def __init__(self) -> None:
        self._encoders: dict[str, Callable[[str], tuple[bytes, int]]] = {
            self.DEFAULT_SCHEME: self._encode_ternary_utf8,
        }
        self._decoders: dict[str, Callable[[bytes, int], str]] = {
            self.DEFAULT_SCHEME: self._decode_ternary_utf8,
        }

    def register_scheme(
        self,
        name: str,
        encoder: Callable[[str], tuple[bytes, int]],
        decoder: Callable[[bytes, int], str],
    ) -> None:
        """Register an encoding pair under a stable scheme name."""
        if not name or name in self._encoders:
            raise ValueError("Scheme name must be nonempty and unused")
        self._encoders[name] = encoder
        self._decoders[name] = decoder

    @staticmethod
    def _encode_ternary_utf8(text: str) -> tuple[bytes, int]:
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

    @staticmethod
    def _decode_ternary_utf8(packed: bytes, pulse: int) -> str:
        if not isinstance(pulse, int) or isinstance(pulse, bool):
            raise TypeError("pulse must be an integer")
        if pulse < 0 or pulse % TRITS_PER_BYTE:
            raise ValueError("pulse must be a non-negative multiple of six")
        packed = bytes(packed)
        expected = (pulse + TRITS_PER_PACKED_BYTE - 1) // TRITS_PER_PACKED_BYTE
        if len(packed) != expected:
            raise ValueError("packed signal length does not match pulse")
        if any(value > MAX_PACKED_VALUE for value in packed):
            raise ValueError("packed signal contains an invalid base-3 group")
        trits: list[int] = []
        for value in packed:
            group = [0] * TRITS_PER_PACKED_BYTE
            for index in range(TRITS_PER_PACKED_BYTE - 1, -1, -1):
                value, group[index] = divmod(value, BASE)
            trits.extend(group)
        if any(trits[pulse:]):
            raise ValueError("packed signal has non-zero trailing padding")
        decoded = bytearray()
        for start in range(0, pulse, TRITS_PER_BYTE):
            value = 0
            for digit in trits[start : start + TRITS_PER_BYTE]:
                value = value * BASE + digit
            if value > 0xFF:
                raise ValueError("ternary group is outside the UTF-8 byte range")
            decoded.append(value)
        try:
            return decoded.decode("utf-8")
        except UnicodeDecodeError as error:
            raise ValueError("signal does not contain valid UTF-8") from error

    def encode(self, text: str, scheme: str | None = None) -> tuple[bytes, int]:
        """Encode text and return packed bytes and valid symbol count."""
        selected = scheme or self.DEFAULT_SCHEME
        try:
            encoder = self._encoders[selected]
        except KeyError as error:
            raise ValueError(f"Unsupported AIVA signal scheme: {selected}") from error
        return encoder(text)

    def default_scheme(self, scheme: str | None = None) -> str:
        """Return the selected scheme name for inclusion in message metadata."""
        selected = scheme or self.DEFAULT_SCHEME
        if selected not in self._encoders:
            raise ValueError(f"Unsupported AIVA signal scheme: {selected}")
        return selected

    def decode(self, signal: bytes, pulse: int, scheme: str | None = None) -> str:
        """Decode a signal with the selected scheme."""
        selected = scheme or self.DEFAULT_SCHEME
        try:
            decoder = self._decoders[selected]
        except KeyError as error:
            raise ValueError(f"Unsupported AIVA signal scheme: {selected}") from error
        return decoder(signal, pulse)
