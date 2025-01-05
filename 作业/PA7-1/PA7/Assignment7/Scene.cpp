//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <functional>


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f shader(Intersection& inter, const Ray& ray, const Scene* scn)
{
	Vector3f ret;
	Intersection pos;
	float pdf;
	scn->sampleLight(pos, pdf);
	auto orig = inter.coords;
	auto dir = pos.coords - orig;
	dir = normalize(dir);
	Ray _ray(orig, dir);
	auto _inter = scn->intersect(_ray);
	if (_inter.happened && _inter.m->hasEmission())
	{
		Vector3f L_i = pos.emit;
		auto fr = inter.m->eval(ray.direction, dir, inter.normal);
		float cos_theta = dotProduct(inter.normal, dir);
		float cos_theta_x = -dotProduct(pos.normal, dir);
		Vector3f x = inter.coords;
		Vector3f p = pos.coords;
		auto L_dir = L_i * fr * cos_theta * cos_theta_x / dotProduct(x - p, x - p) / pdf;

		ret += L_dir;
	}

	if (get_random_float() < scn->RussianRoulette)
	{
		auto wi = inter.m->sample(ray.direction, inter.normal);
        wi = normalize(wi);
		auto _inter = scn->intersect(Ray(inter.coords, wi));
		if (_inter.happened && !_inter.m->hasEmission())
		{
			Vector3f L_indir = shader(_inter, Ray(inter.coords, wi), scn) * inter.m->eval(ray.direction, wi, inter.normal) * crossProduct(wi, inter.normal)
				/ inter.m->pdf(ray.direction, wi, inter.normal) / scn->RussianRoulette;
            ret += L_indir;
		}
	}
	return ret;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    auto inter = intersect(ray);
    if (!inter.happened) return this->backgroundColor;
    if (inter.m->hasEmission())
    {
        return inter.m->getEmission();
    }
    else
    {
        return shader(inter, ray, this);
    }
}