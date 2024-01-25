
from imitrob_templates.templates.MoveUpTask import MoveUpTask
from imitrob_templates.templates.PickTask import PickTask
from imitrob_templates.templates.UnglueTask import UnglueTask
from imitrob_templates.templates.PointTask import PointTask
from imitrob_templates.templates.PourTask import PourTask
from imitrob_templates.templates.PushTask import PushTask
from imitrob_templates.templates.PutIntoTask import PutIntoTask
from imitrob_templates.templates.ReleaseTask import ReleaseTask
from imitrob_templates.templates.StopTask import StopTask
from imitrob_templates.templates.StackTask import StackTask
from imitrob_templates.templates.PassTask import PassTask
from imitrob_hri.imitrob_nlp.nlp_utils import template_name_synonyms


def to_default_name(name, ct='template'):
    name = name.lower()
    name = name.replace("_", " ")
    assert isinstance(name, str), f"name is not string, it is {type(name)}"
    ct_name_synonyms = eval(ct+'_name_synonyms')

    for key in ct_name_synonyms.keys():
        for item in ct_name_synonyms[key]:
            if name == item.lower():
                return ct_name_synonyms[key][0]
    print(f"Exception for {name} not in {ct_name_synonyms}")
    print("returning")

def create_template(template_name, nlp=False):
    template_name = to_default_name(template_name)
    if template_name is None:
        return

    return {
    'stop': StopTask,
    'release': ReleaseTask,
    'move-up': MoveUpTask,
    'pick': PickTask,
    'point': PointTask,
    'push': PushTask,
    'unglue': UnglueTask,
    'put-into': PutIntoTask,
    'pour': PourTask,
    'stack': StackTask,
    'pass': PassTask
    }[template_name](nlp=nlp)
    
    
    
if __name__ == '__main__':
    import time
    tmplts = ['stop',
    'release',
    'move-up',
    'pick',
    'point',
    'push',
    'unglue',
    'put-into',
    'pour',
    'stack',
    'pass']
    
    for tmplt in tmplts:
        t0 = time.perf_counter()
        t = create_template(tmplt)
        print(f"time {tmplt}: {time.perf_counter()-t0}")