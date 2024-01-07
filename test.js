const Emcc = require("./exports/collisions.js");

Emcc['onRuntimeInitialized'] = function() { 
	const collisions = new Emcc.Collisions();
	collisions.addAABB("a", 0, 0, 16, 16);
	const testbox = collisions.b2AABB_ConstructFromCenterSize(-32, 0, 16, 16);
	const res = collisions.test(testbox, 32, 0);
	console.log(res.x, res.y);
};
