#!/usr/bin/env python3
"""
mock_server.py — локальний макет сервера курсу для налагодження ДЗ15.

Відтворює протокол з ТЗ, включно з «поганими» сценаріями, які на
справжньому сервері не викличеш на замовлення:

  T01  → 201 першого разу, 200 при повторі (нормальний шлях)
  T02  → 400 (помилка в даних) — клієнт НЕ має повторювати
  T03  → 401 (хибний ключ)     — клієнт НЕ має повторювати
  T04  → 503 двічі, потім 201  — клієнт має повторити й дотиснути
  T05  → 503 завжди            — клієнт має вичерпати 5 спроб
  T06  → «тиша» 10 с           — перевірка таймауту читання
  решта → 201/200 як звичайно

Запуск:  python3 mock_server.py [порт]
"""
import json
import sys
import time
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

API_KEY = "dz12-vX7mK4qT9r2w"

# Скільки разів уже приходив POST по кожному тесту (для сценаріїв 503→201).
attempts = defaultdict(int)
# Що реально «збережено» на сервері: (testId, studentId) -> payload
storage = {}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        print(f"    [сервер] {fmt % args}")

    def _reply(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path != "/api/dz12/results":
            self._reply(404, {"error": "unknown endpoint"})
            return

        if self.headers.get("x-api-key") != API_KEY:
            self._reply(401, {"error": "bad or missing x-api-key"})
            return

        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length)

        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as exc:
            self._reply(400, {"errors": [f"malformed json: {exc}"]})
            return

        test_id = payload.get("testId", "")
        student = payload.get("studentId", "")

        errors = []
        if not student:
            errors.append("studentId is required")
        if not test_id:
            errors.append("testId is required")
        if "simulation" not in payload:
            errors.append("simulation is required")
        if errors:
            self._reply(400, {"errors": errors})
            return

        attempts[test_id] += 1
        n = attempts[test_id]

        # --- сценарії за testId --------------------------------
        if test_id == "T02":
            self._reply(400, {"errors": ["dropPoints: expected non-empty array"]})
            return
        if test_id == "T03":
            self._reply(401, {"error": "key revoked for this test"})
            return
        if test_id == "T04" and n <= 2:
            self._reply(503, {"error": "temporarily unavailable"})
            return
        if test_id == "T05":
            self._reply(503, {"error": "temporarily unavailable"})
            return
        if test_id == "T06":
            print("    [сервер] T06: імітую тишу 10 с (клієнт має відвалитись по таймауту)")
            time.sleep(10)
            self._reply(201, {"saved": True})
            return

        key = (test_id, student)
        first_time = key not in storage
        storage[key] = payload
        self._reply(201 if first_time else 200, {"saved": True, "testId": test_id})

    def do_GET(self):
        parts = self.path.strip("/").split("/")
        # api/dz12/results/{testId}/{studentId}
        if len(parts) == 5 and parts[:3] == ["api", "dz12", "results"]:
            key = (parts[3], parts[4])
            if key in storage:
                self._reply(200, {"found": True, "testId": parts[3], "studentId": parts[4]})
            else:
                self._reply(404, {"found": False})
            return
        self._reply(404, {"error": "not found"})


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    print(f"Мок-сервер курсу слухає на http://127.0.0.1:{port}")
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
