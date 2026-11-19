# SPDX-License-Identifier: AGPL-3.0-only
"""Fake KiCad API server on a real NNG REP0 socket (pynng), decoding requests with
KiCad's official generated protobufs from `kicad-python` (`kipy`).

Usage: fake_kicad_server.py <version e.g. 10.0.8 | 10.99.0> <ipc://url> [board_filename]

* GetVersion  -> the requested version
* Ping        -> google.protobuf.Empty
* GetOpenDocuments -> one PCB document (board_filename)
* If version >= 10.99: OpenDocument and RunBoardJobExportSvg are "handled" (AS_BAD_REQUEST
  for the empty probe requests, like a real headless server validating parameters);
  everything else -> AS_UNHANDLED.
"""
import sys

import pynng
from google.protobuf import empty_pb2
from google.protobuf.any_pb2 import Any
from kipy.proto.common import envelope_pb2 as env
from kipy.proto.common.commands import base_commands_pb2 as base
from kipy.proto.common.commands import editor_commands_pb2 as editor
from kipy.proto.common.types import base_types_pb2 as types


def main() -> None:
    version, url = sys.argv[1], sys.argv[2]
    board = sys.argv[3] if len(sys.argv) > 3 else "/tmp/fake/board.kicad_pcb"
    major, minor, patch = (int(x) for x in version.split("."))
    new_api = (major, minor) >= (10, 99)

    def respond(code, message="", payload=None):
        resp = env.ApiResponse()
        resp.header.kicad_token = "fake-token"
        resp.status.status = code
        resp.status.error_message = message
        if payload is not None:
            resp.message.Pack(payload)
        return resp.SerializeToString()

    with pynng.Rep0(listen=url, recv_timeout=60000) as sock:
        print("READY", flush=True)
        while True:
            try:
                raw = sock.recv()
            except pynng.Timeout:
                return
            req = env.ApiRequest()
            req.ParseFromString(raw)
            name = req.message.type_url.rsplit("/", 1)[-1]
            print(f"request {name} from {req.header.client_name!r}", file=sys.stderr, flush=True)
            if name == "kiapi.common.commands.GetVersion":
                v = types.KiCadVersion(major=major, minor=minor, patch=patch, full_version=version)
                out = respond(env.AS_OK, payload=base.GetVersionResponse(version=v))
            elif name == "kiapi.common.commands.Ping":
                out = respond(env.AS_OK, payload=empty_pb2.Empty())
            elif name == "kiapi.common.commands.GetOpenDocuments":
                q = editor.GetOpenDocuments()
                req.message.Unpack(q)
                docs = []
                if q.type == types.DOCTYPE_PCB:
                    d = types.DocumentSpecifier(type=types.DOCTYPE_PCB, board_filename=board)
                    d.project.name = "fake"
                    d.project.path = "/tmp/fake"
                    docs.append(d)
                out = respond(env.AS_OK, payload=editor.GetOpenDocumentsResponse(documents=docs))
            elif new_api and name in (
                "kiapi.common.commands.OpenDocument",
                "kiapi.board.jobs.RunBoardJobExportSvg",
            ):
                out = respond(env.AS_BAD_REQUEST, "no document specified")
            else:
                out = respond(env.AS_UNHANDLED, f"no handler available for request of type {name}")
            sock.send(out)


if __name__ == "__main__":
    main()
