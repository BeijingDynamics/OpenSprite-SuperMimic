"""Apply the actor-only multiplier after checkpoint optimizer restoration."""
def set_learning_rates(learners,base_rate,actor_scale):
    for optimizer in [learners[0].opt_wm,*[item.opt_v for item in learners]]:
        for group in optimizer.param_groups:group['lr']=base_rate
    for learner in learners:
        for group in learner.opt_pi.param_groups:group['lr']=base_rate*actor_scale
