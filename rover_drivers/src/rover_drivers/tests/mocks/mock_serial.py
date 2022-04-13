from unittest import mock
from unittest.mock import patch as mock_patch


class MockSerial(mock.MagicMock):
    def __init__(self, *args, **kwargs):

        super().__init__()
