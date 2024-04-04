#include <stdlib.h>

#ifndef FSM_EVENT_TYPE
#define FSM_EVENT_TYPE unsigned char
#endif


typedef unsigned char fsm_state_t;
#define FSM_STATE_INIT (0x00)
#define FSM_STATE_FAIL (0xFF)

typedef FSM_EVENT_TYPE fsm_event_t;

#define FSM_ANY_EVENT (NULL)
#define FSM_NOOP (NULL)

struct fsm_t;
typedef fsm_state_t (*action_callback)(struct fsm_t *fsm);
typedef void (*before_action_callback)(struct fsm_t *fsm);
typedef int (*accept_event)(fsm_event_t event);

struct fsm_row_t {
    fsm_state_t current;
    fsm_event_t event;
    // accept_event event;
    // before_action_callback before_enter;
    action_callback action_enter;
    // action_callback action_exit;
    // fsm_state_t next;
};

struct fsm_t {
    struct fsm_row_t *current_transition;
    struct fsm_row_t *stt;// state transition table;
    void *priv;
    fsm_state_t current;
    fsm_event_t event;
    int stt_size;
};

inline void* fsm_private(struct fsm_t *fsm){
    return fsm->priv;
}
inline fsm_event_t fsm_current_event(struct fsm_t *fsm) {
    return fsm->event;
}
/* internal */
void fsm_transition(struct fsm_t *fsm, struct fsm_row_t *transition)
{
    // if (fsm->current_transition != NULL && fsm->current_transition->action_exit) {
    //     fsm->current_transition->action_exit();
    // }
    fsm->current_transition = transition;
    if (fsm->current_transition->action_enter) {
        fsm->current_transition->action_enter(fsm);
    }
}

int fsm_process_event(struct fsm_t *fsm, fsm_event_t event) {
    // if in state X and event Y then do transition to Z
    fsm->event = event;
    for (struct fsm_row_t *row = fsm->stt; row != NULL; ++row)
    {
        if (row->current != fsm->current)
        {
            continue;
        }

        if (row->event && !(row->event(event)))
        {
            continue;
        }

        fsm_transition(fsm, row);

        return 0;
    }

    /* unhandled state */
    fsm->current = FSM_STATE_FAIL;
    return -1;
}

void fsm_init(struct fsm_t *fsm, struct fsm_row_t stt[]) 
{
    fsm->current = FSM_STATE_INIT;
    fsm->current_transition = NULL;
    fsm->stt = stt;
    fsm->stt_size = sizeof(*stt);
}

void fsm_free()
{
}
