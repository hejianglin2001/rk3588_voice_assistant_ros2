# TaskCommand.msg — 认知层(llm_node)下发给决策层(decision_node)的任务命令
# 解耦 LLM 与执行层：llm_node 不再直接调 YOLO，而是发结构化命令交给决策节点路由

string action   # "detect" 视觉检测 / "chat" 对话
string target   # detect: 目标物体名(英文); chat: 完整回复文本
string raw      # 用户原始输入（供 detect 结果回填时生成自然回答）
