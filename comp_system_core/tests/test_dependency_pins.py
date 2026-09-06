"""
Check that pyproject.toml would install the versions we actually tested against.

WHY THIS EXISTS
---------------
The robot died on the SystemCore with, inside phoenix6's own code:

    AttributeError: type object 'wpiutil._wpiutil.SendableRegistry' has no attribute 'addLW'

Nothing was wrong with our code.  pyproject.toml said "phoenix6>=26.1", and pip REFUSES
to install a pre-release unless the specifier itself names one - so the robot got 26.3.0,
the 2026 build, which calls SendableRegistry.addLW().  WPILib 2027 removed that method.

The development machine had 26.50.0a1 because it was installed by hand with an explicit
==, so every local test passed while the file that actually provisions the robot pointed
somewhere else.  The sim cannot catch this: it never reads pyproject.toml.

So the rule this file enforces is: if the version we are developing against is a
pre-release, the pin has to admit pre-releases, or the robot silently gets a different
package than the one we tested.
"""

import pathlib
import tomllib

import pytest
from packaging.requirements import Requirement
from packaging.version import Version

import importlib.metadata as md


PYPROJECT = pathlib.Path(__file__).resolve().parent.parent / 'pyproject.toml'


def _requires():
    cfg = tomllib.loads(PYPROJECT.read_text(encoding='utf-8-sig'))
    return cfg['tool']['robotpy']


@pytest.mark.parametrize('spec', _requires()['requires'])
def test_pin_matches_what_is_installed(spec):
    """Every requires= entry must resolve to the version this machine is testing with."""
    req = Requirement(spec)
    try:
        installed = md.version(req.name)
    except md.PackageNotFoundError:
        pytest.fail(f'{req.name} is in pyproject.toml requires= but is not installed here, '
                    f'so nothing has ever tested against it')

    assert req.specifier.contains(installed, prereleases=True), (
        f'{req.name} {installed} is installed but does not satisfy {spec!r}')

    if Version(installed).is_prerelease and not req.specifier.prereleases:
        pytest.fail(
            f'{spec!r} will NOT install {req.name} {installed}.\n'
            f'pip only accepts pre-releases when the specifier names one, so the robot gets\n'
            f'the newest STABLE release instead - a different package than the one tested\n'
            f'here.  This is exactly the phoenix6 26.3.0 / SendableRegistry.addLW failure.\n'
            f'Fix: write the pin as >={installed} rather than a stable-looking bound.')


def test_robotpy_version_matches_the_installed_core():
    """robotpy_version provisions the robot.  If it disagrees with what we test against,
    the robot runs a different WPILib than the one these tests passed on."""
    declared = _requires()['robotpy_version']
    installed = md.version('robotpy')
    assert declared == installed, (
        f'pyproject.toml declares robotpy_version = {declared!r} but {installed!r} is '
        f'installed here.  The robot would run a different core than we tested.')
