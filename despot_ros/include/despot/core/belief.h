#ifndef BELIEF_H
#define BELIEF_H

#include <vector>

#include <despot/util/random.h>
#include <despot/util/logging.h>
#include <despot/core/history.h>

namespace despot {

class State;
class StateIndexer;
class DSPOMDP;

/* =============================================================================
 * Belief class
 * =============================================================================*/

class Belief {
public:
	const DSPOMDP* model_;
	History history_;

public:
	Belief(const DSPOMDP* model);
	virtual ~Belief();

	virtual std::vector<State*> Sample(int num) const = 0;
	virtual void Update(int action, OBS_TYPE obs) = 0;
	
	virtual std::string text() const;
	friend std::ostream& operator<<(std::ostream& os, const Belief& belief);
	virtual Belief* MakeCopy() const = 0;

	virtual std::map<std::string, double> get_pdf();

	virtual double get_max();

	virtual void propagate_values() = 0;

	virtual void restart() = 0;

	virtual void change_values(Belief *b)=0;

	virtual void propagate();
	virtual std::vector<State*> get_propagation();
	virtual std::vector<State*> get_particles();
	virtual double particles_sum(std::vector<State*> particles);

	static std::vector<State*> Sample(int num, std::vector<State*> belief,
		const DSPOMDP* model);
	static std::vector<State*> Resample(int num, const std::vector<State*>& belief,
		const DSPOMDP* model, History history, int hstart = 0);
	static std::vector<State*> Resample(int num, const Belief& belief,
		History history, int hstart = 0);
	static std::vector<State*> Resample(int num, const DSPOMDP* model,
		const StateIndexer* indexer, int action, OBS_TYPE obs);
};

/* =============================================================================
 * ParticleBelief class
 * =============================================================================*/

class ParticleBelief: public Belief {
protected:

	Belief* prior_;
	bool split_;
	std::vector<State*> initial_particles_;
	const StateIndexer* state_indexer_;
	std::vector<State*> particles_;
	int num_particles_;

	int text_to_id(std::string text);

public:

	ParticleBelief(std::vector<State*> particles, const DSPOMDP* model,
		Belief* prior = NULL, bool split = true);

	virtual ~ParticleBelief();
	void state_indexer(const StateIndexer* indexer);

	virtual const std::vector<State*>& particles() const;
	virtual std::vector<State*> Sample(int num) const;

	virtual void Update(int action, OBS_TYPE obs);

	virtual Belief* MakeCopy() const;

	virtual std::string text() const;

	virtual void change_values(Belief *b);

	virtual std::map<std::string, double> get_pdf();
	virtual std::vector<State*> get_particles();
	virtual double particles_sum(std::vector<State*> particles);

	virtual std::vector<State*> get_propagation();

	virtual void propagate();
	virtual double get_max();

	virtual void propagate_values();

	virtual void restart();
};

} // namespace despot

#endif
