.. _state_wait_for:
*******
WaitFor
*******

| This state allows you to control the flow of the task, i.e. you can pause for a given amount of time or wait until a given topic is active.

.. note::
  This is a blocking state, i.e. the state machine won't transition to the next step until the condition is met.

Outcomes
########

| This state has two outcomes: :code:`success` (green socket) and :code:`failure` (red socket). If the state is waiting for a topic to be active but the latter is not, return :code:`failure` after a given amount of time. If the topic has been detected or the amount of time to wait (without topic) has been reached, return :code:`success`.

How to configure
################

| Once dropped in the task editor, you should see two parameters appear in the state:
* **topic_name**: Name (string) of the topic the state will wait for. If left empty, the state behaves like a standard :code:`wait` if :code:`timeout` is not set to :code:`None`
* **timeout**: Amount of time (in seconds) the state has to wait for. If :code:`topic_name` is not empty, the state will return :code:`failure` if a message is not published by the topic within this amount of time. Otherwise, wait for the given amount of time.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_wait_for>`.
