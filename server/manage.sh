#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
exec .venv/bin/python manage_service.py "${1:-status}"
