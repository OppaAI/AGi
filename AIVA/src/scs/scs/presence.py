"""Send an authenticated-tailnet presence heartbeat to AuRoRA's Web Teleops Console."""
from __future__ import annotations

import json
import logging
import os
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

_LOG = logging.getLogger("aiva.presence")


def main() -> None:
    """Keep AIVA marked online while this small companion process is alive."""
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    endpoint = os.getenv("AGI_WEB_TELEOPS_CONSOLE_PRESENCE_URL", "").strip()
    if not endpoint:
        raise SystemExit("Set AGI_WEB_TELEOPS_CONSOLE_PRESENCE_URL to AuRoRA's Tailscale Web Teleops Console presence endpoint")
    interval = max(2.0, float(os.getenv("AGI_PRESENCE_INTERVAL", "4")))
    body = json.dumps({"source": "AIVA"}).encode("utf-8")
    _LOG.info("Sending AIVA presence to %s every %.1f seconds", endpoint, interval)
    while True:
        request = Request(endpoint, data=body, headers={"Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=3) as response:
                if response.status not in (200, 204):
                    _LOG.warning("Web Teleops Console presence returned HTTP %s", response.status)
        except (OSError, URLError, TimeoutError) as error:
            _LOG.warning("Web Teleops Console presence unavailable; retrying: %s", error)
        time.sleep(interval)


if __name__ == "__main__":
    main()
