import sys
from pprint import pprint
from pathlib import Path

# print(Path(__file__).resolve().parents[1])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pprint(sys.path)

from utils.base_dataclasses import QuaternionMsgResponse

a = QuaternionMsgResponse(1,1,1,1,1)

print(a)


