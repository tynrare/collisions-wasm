#include <math.h>
#include "box2d/types.h"
#include "box2d/aabb.h"
#include "box2d/math.h"
#include <map>
#include <vector>
#include <string>
#include <stdlib.h>

class Collisions {
	private:
		std::map<std::string, b2AABB*> b2AABBs;
		int extend_margin;
	public:
	Collisions(int extend_margin = 1) {
		this->extend_margin = extend_margin;
	};
	
	inline void b2AABB_setPos(b2AABB *a, float x, float y) {
		b2Vec2 extents = b2AABB_Extents(*a);
		a->lowerBound.x = x - extents.x;
		a->lowerBound.y = y - extents.y;
		a->upperBound.x = x + extents.x;
		a->upperBound.y = y + extents.y;
	}

	inline b2AABB *b2AABB_ConstructFromCenterSizeP(float x, float y, float w, float h) {
		float halfwidth = w / 2;
		float halfheight = h / 2;

		b2AABB *aabb = (b2AABB*)malloc(sizeof(b2AABB));
		aabb->lowerBound.x = x - halfwidth;
		aabb->lowerBound.y = y - halfheight;
		aabb->upperBound.x = x + halfwidth;
		aabb->upperBound.y = y + halfheight;

		return aabb;
	}
	
	inline b2AABB b2AABB_ConstructFromCenterSize(float x, float y, float w, float h) {
		float halfwidth = w / 2;
		float halfheight = h / 2;

		b2AABB aabb = { { x - halfwidth, y - halfheight },  { x + halfwidth, y + halfheight } };

		return aabb;
	}
	
	inline b2AABB b2AABB_ExtendBySize(b2AABB &a, float w, float h)
	{
		b2AABB c;
		c.lowerBound.x = a.lowerBound.x - w / 2;
		c.lowerBound.y = a.lowerBound.y - h / 2;
		c.upperBound.x = a.upperBound.x + w / 2;
		c.upperBound.y = a.upperBound.y + h / 2;
		return c;
	}
	
	inline b2Vec2 sweptAABBCollision(int *iteration, b2Vec2 &pos, b2Vec2 &target, std::vector<b2AABB>& colliders, int extend_margin) {
		*iteration += 1;

		// this whole process has to be recursive
		b2RayCastOutput raycast_closest = { 0 }; // closest ro;
		for (auto &box : colliders) {

			// does not work from inside
			b2RayCastOutput ro = b2AABB_RayCast(box, pos, target);

			// main issue right now: ray clips on corners
			if (ro.hit) {
				if (!raycast_closest.hit || raycast_closest.fraction > ro.fraction) {
					raycast_closest = ro;
				}
			}
		}

		b2Vec2 newpos = { 0, 0 };

		newpos.x = raycast_closest.hit ? raycast_closest.point.x + raycast_closest.normal.x * extend_margin : target.x;
		newpos.y = raycast_closest.hit ? raycast_closest.point.y + raycast_closest.normal.y * extend_margin : target.y;

		if (!raycast_closest.hit) {
			return newpos;
		} 

		b2Vec2 dir = { target.x - newpos.x, target.y - newpos.y };
		b2Vec2 crossnormal = { raycast_closest.normal.y, raycast_closest.normal.x }; // swapped axes
		float newdirlen = b2Dot(dir, crossnormal);
		b2Vec2 newdir = b2MulSV(newdirlen, crossnormal);
		b2Vec2 newtarg = b2Add(newpos, newdir);

		if (newdirlen != 0) {
			return sweptAABBCollision(iteration, newpos, newtarg, colliders, extend_margin);
		}
	   
		return newpos;
	}
	
	inline b2Vec2 simpleAABBCollision(b2AABB &collider, std::vector<b2AABB> &colliders, int extend_margin) {
		b2AABB a = collider;
		b2Vec2 extents = b2AABB_Extents(collider);
		b2Vec2 pos = b2AABB_Center(collider);
		for(auto &b : colliders) {
			//b2Vec2 bpos = b2AABB_Center(b);
			//b2Vec2 bextents = b2AABB_Extents(b);

			b2Vec2 d1 = { b.lowerBound.x - (pos.x + extents.x), b.lowerBound.y - (pos.y + extents.y) };
			b2Vec2 d2 = { (pos.x - extents.x) - b.upperBound.x, (pos.y - extents.y) - b.upperBound.y };

			if (d1.x > 0.0f || d1.y > 0.0f)
				continue;

			if (d2.x > 0.0f || d2.y > 0.0f)
				continue;

			int x = -B2_MAX(d1.x, d2.x);
			int y = -B2_MAX(d1.y, d2.y);
			int normal_y = d1.y > d2.y ? 1 : -1;
			int normal_x = d1.x > d2.x ? 1 : -1;

			if (x < y) {
				pos.x -= x * normal_x;
			}
			else if (x > y) {
				pos.y -= y * normal_y;
			}
			else {
				pos.x -= x * normal_x;
				pos.y -= y * normal_y;
			}
		}

		return pos;
	}
	
	inline b2Vec2 test(	b2AABB *a, float tx, float ty) {
		b2Vec2 target = { tx, ty };
		b2Vec2 extents = b2AABB_Extents(*a);
		b2AABB box_at_goal = b2AABB_ConstructFromCenterSize(target.x, target.y, extents.x * 2, extents.y * 2);
		b2AABB boardphase = b2AABB_Union(*a, box_at_goal);

		// tooptimize
		std::vector<b2AABB> colliders;
		std::vector<b2AABB> colliders_extended;
		for (auto &e : b2AABBs) {
			b2AABB *box = e.second;
			if (box == a) {
				continue;
			}

			if (b2AABB_Overlaps(boardphase, *box)) {
				b2AABB extended_box = b2AABB_ExtendBySize(*box, extents.x * 2 - extend_margin * 2, extents.y * 2 - extend_margin * 2);
				colliders_extended.push_back(extended_box);
				colliders.push_back(*box);
			}
		}
		
		b2Vec2 newpos = {0};
		// push out of any bounds
		newpos = simpleAABBCollision(*a, colliders, extend_margin);
		// swept move
		int iterations = 0;
		newpos = sweptAABBCollision(&iterations, newpos, target, colliders_extended, extend_margin);
		
		return newpos;
	}
	
	inline b2AABB *addAABB(const char *id, float x, float y, float w, float h) {
		b2AABB *aabb = b2AABB_ConstructFromCenterSizeP(x, y, w, h);
		const auto it = b2AABBs.insert({std::string(id), aabb});
		
		return aabb;
	}
	
	/**
	* @returns true if memory was cleared
	*/
	inline bool eraseAABB(const char *id) {
		auto it = b2AABBs.find(std::string(id));
		if (it != b2AABBs.end()) {
			b2AABBs.erase(it);
			free(it->second);

			return true;
		}

		return false;
	}
	
	inline void clear() {
		for (auto &e : b2AABBs) {
			free(e.second);
		}

		b2AABBs.clear();
	}

	inline void freeP(void* p) {
		free(p);
	}
};