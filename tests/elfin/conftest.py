import pytest

from tools.elfin_mock.server import MockServer
from tools.elfin_mock.sim import MockArm, MockConfig


@pytest.fixture
def mock_factory():
    servers = []

    def make(**cfg):
        srv = MockServer(MockArm(MockConfig(**cfg)), cmd_port=0, state_port=0).start()
        servers.append(srv)
        return srv

    yield make
    for s in servers:
        s.stop()


@pytest.fixture
def mock(mock_factory):
    return mock_factory()
