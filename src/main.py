import asyncio
from viam.module.module import Module
try:
    from models.vesc import Vesc
except ModuleNotFoundError:
    # when running as local module with run.sh
    from .models.vesc import Vesc


if __name__ == '__main__':
    asyncio.run(Module.run_from_registry())
