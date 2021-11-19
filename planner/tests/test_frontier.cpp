#include "core/base.h"
#include "utils/frontier.h"
#include "core/hash.h"

int main()
{
	PP_INIT;

	auto compare = [](std::pair<int, int> lhs, std::pair<int, int> rhs) { return lhs.first < rhs.first; };
	auto hash = [](const std::pair<int, int>& pair) { std::hash<int> hasher; return hasher(pair.second); };
	auto equal = [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) { return lhs.second == rhs.second; };
	Planner::Frontier<std::pair<int, int>, decltype(compare), decltype(hash), decltype(equal)> frontier(compare, 100, hash, equal);

	frontier.Push({ -3, 10 });
	frontier.Push({ -4, 10 });
	frontier.Push({ 1, 2 });
	frontier.Push({ -1, 15 });
	frontier.Push({ -5, 12 });
	frontier.Push({ 2, 2 });
	frontier.Push({ 1, 3 });
	frontier.Push({ -2, 14 });
	frontier.Push({ 0, 15 });
	frontier.Push({ -6, 13 });
	frontier.Push({ 1, 1 });
	frontier.Push({ 1, 2 });
	frontier.Push({ 1, 14 });
	frontier.Push({ 1, 13 });
	frontier.Push({ 2, 4 });

	const std::pair<int, int>* previous = nullptr;
	for (auto& e : frontier) {
		if (previous)
			assert(previous->first <= e.first);
		previous = &e;
	}

	auto top = frontier.Pop();
	assert(top.first == 2 && top.second == 4);
	assert((bool)frontier.Find(top) == false);

	assert(frontier.Remove({ 2, 4 }) == 0);
	assert(frontier.Remove({ 0, 1 }) == 1);
	assert(frontier.Remove({ 1, 1 }) == 0);

	for (auto& e : frontier) {
		PP_INFO("{}, {}", e.first, e.second);
	}

	assert((bool)frontier.Find({ 0, 2 }) == true);
	assert((bool)frontier.Find({ 0, 0 }) == false);
	assert((bool)frontier.Find({ 1, 2 }) == true);
	auto test = frontier.Find({ 0, 2 });
	assert(test->first == 1 && test->second == 2);
}
