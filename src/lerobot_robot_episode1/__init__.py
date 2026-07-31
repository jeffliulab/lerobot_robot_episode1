# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified by Jeff (2026): ported from enpeizhao/lerobot_single_student — renamed enpei→episode1, adapted to lerobot 0.6.x plugin API

"""lerobot_robot_episode1：Episode1 机器人套件的官方 lerobot 第三方插件。

官方 `register_third_party_plugins()` 会 import 本包一次；
这里导出两个 config 类即完成 `episode1_follower` / `episode1_leader` 的注册。
"""

from .robots.episode1_follower import Episode1Follower, Episode1FollowerConfig
from .teleoperators.episode1_leader import Episode1Leader, Episode1LeaderConfig

__all__ = [
    "Episode1Follower",
    "Episode1FollowerConfig",
    "Episode1Leader",
    "Episode1LeaderConfig",
]
