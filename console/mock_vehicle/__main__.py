"""Run the fake vehicle: python -m console.mock_vehicle --port 8765."""

import argparse
import asyncio
from pathlib import Path

from console.mock_vehicle import server


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fake vehicle for the operator console (console/1)."
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="use 0.0.0.0 to accept other devices"
    )
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--web-root", type=Path, default=Path(__file__).resolve().parent.parent / "web"
    )
    args = parser.parse_args()
    try:
        asyncio.run(server.main(args.host, args.port, args.web_root))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
