/////// 노드 설명 //////
1. pub_index : 라이다로 부터 인덱스 받기 전에 실험해보려고 임의로 인덱스를 주는 노드.
2. pub_test : Select Goal Point 를 담당하는 노드로 인덱스를 받아서 goal/init 포인트를 Generate Path 노드에 publish.
3. dubin_test : Generate Path 를 담당하는 노드로 실제 Path 를 생성하여 control 노드로 publish.
4. control_test : path 를 받아서 CARLA 에게 vehicle cmd 를 publish.
///////////////////////