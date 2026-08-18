# VisionContext.msg — 画面物体上下文（YOLO 检测结果摘要，喂给 LLM 做视觉问答）
# 由 yolo_node 在检测期间持续发布，llm_node 保存最近一帧作为视觉上下文

string objects   # 逗号分隔，如 "person(87),cup(92)"，括号内为置信度百分比
