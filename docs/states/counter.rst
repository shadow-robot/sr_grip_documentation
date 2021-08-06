.. _state_counter:
*******
Counter
*******

| This state allows you to increment/decrement a variable by 1 everytime the state is executed. It can be used as a :code:`for loop`, for instance if some instructions should be repeated a given number of times.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`finished` (purple socket). During execution, if the current value of the internal counter of the state is **not** equal to the target value, then the outcome :code:`success` will be returned. The second outcome, :code:`finished` will be returned if and only if the current value of the counter **is equal** to the target value.

How to configure
################

| Once dropped in the task editor, you should see three parameters appear in the state:
* **initial_value**: Value the created counter will take the first time the state is executed. **It must be an integer**.
* **end_value**: Target value for the counter (must be an integer as well). If this value is bigger than :code:`initial_value`, the value of the counter will be **incremented**. If it is smaller, the value of hte counter will be decremented. **Is set to :code:`None`, you will create an infinite loop**. So, please be careful when using :code:`None` for this parameter.
* **output**: Name you want to give to the counter of this state. It must be a string.

| The value of the counter will also be stored in the :code:`userdata` of the state machine. This means that other states will be able to access its value, if referenced by name.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_counter>`.
