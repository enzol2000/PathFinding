#include "PathSearch.h"

namespace fullsail_ai { namespace algorithms {

	PathSearch::PathSearch()
	{
	}

	PathSearch::~PathSearch()
	{
	}

	void PathSearch::initialize(TileMap* _tileMap)
	{
		tileMap = _tileMap;
		Run = false;
		hWeight = 1.2f;
		
		for (size_t i = 0; i < tileMap->getColumnCount(); i++)
		{
			for (size_t j = 0; j < tileMap->getRowCount(); j++)
			{
				Tile* Ntile = tileMap->getTile(j, i);
				if (Ntile)
				{
					if (Ntile->getWeight() > 0)
					{
						SearchNode* GivenNode = new SearchNode(Ntile);
						nodes[Ntile] = GivenNode;
					}
				}
			}
		}


		
		// making a node = to null at some point here
		for (size_t i = 0; i < tileMap->getColumnCount(); i++)
		{
			for (size_t j = 0; j < tileMap->getRowCount(); j++)
			{
				Tile* Ntile = tileMap->getTile(j, i);
				SearchNode* node = nullptr;
				if (Ntile)
				{
					if (Ntile->getWeight() > 0)
					{
						node = nodes[Ntile];
					}
				}

				if (node)
				{
					
					SearchNode* neighbor;

					//right
					Ntile = tileMap->getTile(j, i + 1);
					if (Ntile)
					{
						if (Ntile->getWeight() > 0)
						{
							neighbor = nodes[Ntile];

							if (neighbor && neighbor->tile->getWeight() > 0)
							{
								node->neighbors.push_back(neighbor);
							}
						}
					}

					//left
					Ntile = tileMap->getTile(j ,i - 1);
					if (Ntile)
					{
						if (Ntile->getWeight() > 0)
						{
							neighbor = nodes[Ntile];

							if (neighbor && neighbor->tile->getWeight() > 0)
							{
								node->neighbors.push_back(neighbor);
							}
						}
					}

					if (j % 2 == 0)
					{
						//top left
						Ntile = tileMap->getTile(j - 1, i - 1);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

						//down left
						Ntile = tileMap->getTile(j + 1, i - 1);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

						//down right
						Ntile = tileMap->getTile(j + 1, i);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

						//top right
						Ntile = tileMap->getTile(j - 1, i);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

					}
					else
					{
						//top left
						Ntile = tileMap->getTile(j - 1, i);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

						//down left
						Ntile = tileMap->getTile(j + 1, i);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

						//down right
						Ntile = tileMap->getTile(j + 1, i + 1);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

						//top right
						Ntile = tileMap->getTile(j - 1, i + 1);
						if (Ntile)
						{
							if (Ntile->getWeight() > 0)
							{
								neighbor = nodes[Ntile];

								if (neighbor && neighbor->tile->getWeight() > 0)
								{
									node->neighbors.push_back(neighbor);
								}
							}
						}

					}

					
				}

				
			}
		}

	}

	void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
	{
		FinalNode = nodes[tileMap->getTile(goalRow, goalColumn)];
		
		SearchNode* FirstSearch = nodes[tileMap->getTile(startRow, startColumn)];
		PlannerNode* FirstPlanner = new PlannerNode(FirstSearch);

		FirstPlanner->givenCost = 0;	
		FirstPlanner->heuristicCost = HeCost(FirstSearch, FinalNode);
		FirstPlanner->finalCost = FirstPlanner->heuristicCost * hWeight;
		FirstPlanner->parent = nullptr;

		visited[FirstSearch] = FirstPlanner;
		open.push(FirstPlanner);
	}

	void PathSearch::update(long timeslice)
	{
		tileMap->resetTileDrawing();
		while (!open.empty())
		{
			PlannerNode* current = open.front();
			open.pop();

			if (current->searchNode == FinalNode)
			{
				Run = true;
				for (auto it = visited.begin(); it != visited.end(); it++)
				{
					it->first->tile->setFill(0xFF0000FF);
				}
				while (current)
				{

					solution.push_back(current->searchNode->tile);
					current = current->parent;
				}
				return;
			}
			
			for (size_t i = 0; i < current->searchNode->neighbors.size(); i++)
			{
				SearchNode* successor = current->searchNode->neighbors[i];

				float tempGivenCost = current->givenCost + successor->tile->getWeight();

				if (visited[successor] != NULL)
				{
					PlannerNode* successorNode = visited[successor];
					if (tempGivenCost < successorNode->givenCost)
					{
						open.remove(successorNode);
						successorNode->givenCost = tempGivenCost;
						successorNode->finalCost = successorNode->givenCost + successorNode->heuristicCost * hWeight;
						successorNode->parent = current;
						open.push(successorNode);
					}
				}
				else
				{
					PlannerNode* successorNode = new PlannerNode(successor);
					successorNode->givenCost = tempGivenCost;
					successorNode->heuristicCost = HeCost(successor, FinalNode);
					successorNode->finalCost = successorNode->givenCost + successorNode->heuristicCost * hWeight;
					successorNode->parent = current;

					open.push(successorNode);
					visited[successor] = successorNode;

					// show next neighbors to go filling them with green

					PlannerNode* lines = successorNode;

					for (size_t i = 0; i < lines->searchNode->neighbors.size(); i++)
					{
						successorNode->searchNode->neighbors[i]->tile->setFill(0xFF00FF00);
					}

					// make a red line from succesor to beginning
					while (lines->parent)
					{
						Tile* par = lines->parent->searchNode->tile;
						lines->searchNode->tile->addLineTo(par, 0xFFFF0000);
						lines = lines->parent;
					}
				}
			}

			/*
			PriorityQueue<PlannerNode*, Compr>iterate = open;
			//std::queue<PlannerNode*>iterate = open;

			for (size_t i = 0; i < iterate.size(); i++)
			{
				PlannerNode* node = iterate.front();
				iterate.pop();
				//node->searchNode->tile->setFill(0xFF00FF00);
			}
			*/
			
			for (auto it = visited.begin(); it != visited.end(); it++)
			{
				it->first->tile->setFill(0xFF0000FF);
			}
			if (timeslice == 0)
			{
				break;
			}
		}
		
	}

	void PathSearch::exit()
	{
		FinalNode = nullptr;
		Run = false;

		solution.clear();

		while (!open.empty())
		{
			open.pop();
		}
		visited.clear();
	}

	void PathSearch::shutdown()
	{
		FinalNode = nullptr;
		Run = false;
		hWeight = NULL;
		solution.clear();

		while(!open.empty())
		{
			open.pop();
		}

		for (auto it = visited.begin(); it != visited.end();)
		{
			if (it->second)
			{
				delete it->second;
			}
			it = visited.erase(it);
		}

		for (auto it = nodes.begin(); it != nodes.end();)
		{
			if (it->second)
			{
				delete it->second;
			}
			it = nodes.erase(it);
		}
		visited.clear();
		nodes.clear();
	}

	bool PathSearch::isDone() const
	{
		return Run;
	}

	std::vector<Tile const*> const PathSearch::getSolution() const
	{
		return solution;
	}
}}  // namespace fullsail_ai::algorithms

