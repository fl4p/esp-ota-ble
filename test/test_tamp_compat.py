#!/usr/bin/env python3
"""Pin legacy-wire selection across tamp encoder APIs; no radio required."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
import esp_ota_ble as O

class TampCompatibility(unittest.TestCase):
    def test_extended_default_is_disabled(self):
        calls=[]
        def compress(data,*,window,extended=True):
            calls.append((data,window,extended))
            return b'\x9aextended' if extended else b'\x98legacy'
        with patch.dict(sys.modules,tamp=SimpleNamespace(compress=compress)):
            self.assertEqual(O.build_tamp_payload(b'image'),b'\x98legacy')
        self.assertEqual(calls,[(b'image',12,False)])

    def test_legacy_api_without_keyword(self):
        calls=[]
        def compress(data,*,window):
            calls.append((data,window));return b'\x98legacy'
        with patch.dict(sys.modules,tamp=SimpleNamespace(compress=compress)):
            self.assertEqual(O.build_tamp_payload(b'image'),b'\x98legacy')
        self.assertEqual(calls,[(b'image',12)])

    def test_unrelated_type_error_is_not_retried(self):
        calls=[]
        def compress(*args,**kwargs):
            calls.append(kwargs);raise TypeError('compressor internal failure')
        with patch.dict(sys.modules,tamp=SimpleNamespace(compress=compress)):
            with self.assertRaisesRegex(TypeError,'internal failure'):
                O.build_tamp_payload(b'image')
        self.assertEqual(len(calls),1)

    def test_empty_and_incompatible_headers_fail(self):
        for output in [b'',None,b'\x9aextended',b'\x99header',b'\xffbad']:
            with self.subTest(output=output):
                with patch.dict(sys.modules,tamp=SimpleNamespace(compress=lambda *a,**k:output)):
                    with self.assertRaises(O.OtaBleError):O.build_tamp_payload(b'image')

    def test_missing_dependency_remains_an_import_error(self):
        with patch.dict(sys.modules,tamp=None):
            with self.assertRaises(ImportError):O.build_tamp_payload(b'image')

if __name__=='__main__':unittest.main()
