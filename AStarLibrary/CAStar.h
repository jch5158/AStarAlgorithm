#pragma once

#include <iostream>
#include <Windows.h>
#include <list>

class CAStar
{
public:

	CAStar(int mapWidth, int mapHeight);

	~CAStar(void);

	struct stRouteNode
	{
		bool bUseFlag;
		int x;
		int y;
	};

	enum class eNodeAttribute
	{
		NODE_UNBLOCK,
		NODE_BLOCK,
		NODE_START_POINT,
		NODE_DESTINATION_POINT
	};

	bool PathFind(int startX, int startY, int destinationX, int destinationY, stRouteNode routeNodeArray[], int routeNodeArraySize);

	bool SetMapAttribute(int x, int y, eNodeAttribute nodeAttribute);

	void ResetMapAttribute(void);

private:

	CAStar(void);

	enum class eNodeState
	{
		NODE_NONE,
		NODE_OPENED,
		NODE_CLOSED
	};

	struct stNode
	{
		int x;
		int y;

		// 블럭 이동 횟수
		float G;

		// 목적지까지의 블럭의 개수
		float H;

		// G+H 값
		float F;

		eNodeState nodeState;

		eNodeAttribute nodeAttribute;

		stNode* pParentNode;
	};

	class CSortAscendingOrder
	{
	public:

		CSortAscendingOrder(void);

		~CSortAscendingOrder(void);

		bool operator()(const stNode* pLeft, const stNode* pRight) const;
	};

	void createSurroundNode(stNode* pNode);

	void createNode(int x, int y, stNode* pParentNode);

	stNode* getExplorationNodeFromOpenList(void);	

	bool setRouteArray(stNode* pDestNode, stRouteNode routeNodeArray[], int routeNodeArraySize);
	bool makeBresenhamLine(int startX, int startY, int endX, int endY);
	void makeOptimizePath(stNode* pNode);

	void resetNodeState(void);

	stNode** mMap;

	int mMapWidth;
	int mMapHeight;

	stNode mDestinationNode;

	std::list<stNode*> mOpenList;
};

