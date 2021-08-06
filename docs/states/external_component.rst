.. _state_external_component:
****************
Generated states
****************

| All the generated states corresponding to external method integrated to GRIP have the same interface. Whether it is a high level method (e.g. grasping algorithm), motion planner, or even controller, the generated state allows you to configure the content of the service/action you have interfaced to the framework.

Outcomes
########

| This state has at least two outcomes: :code:`failure` (red socket) and at least one outcome output by the action/service server. The number of outcomes depends then on how many different outcomes your server can return. The outcome :code:`failure` is returned when the action/service server cannot be reached or when the configuration of the state is not correct.

How to configure
################

| Once dropped in the task editor, you should see four parameters appear in the state:
* **input**: Input value you want to set the :code:`input` field of your server request. For more information, see :ref:`here <wrap_code>`. If you want to set a string as an input, please write the content of the string between **double quotes**. Otherwise, the value of this field will be either looked for in the userdata or a manager.
* **input_type**: Name of the manager from which the input should be retrieved. Please use the values available in the dropdowm menu. **If left empty**, the input will be looked into the userdata.
* **output**: Name to give to the object returned by the server.
* **output_type**: Name of the manager to which the output of the server should be send. Please use the values available in the dropdowm menu. **If left empty** stores the output object in the userdata. If :code:`output` **is also empty** does not store the output object.

.. note::
  | If you want to have a list as an input of your service or action, you can set :code:`input` as :code:`[1,2,3]` or :code:`[a, b]`.
  | If you want the input of your service to be the string :code:`close`, then the slot :code:`input` must be set to :code:`"close"`. If you type :code:`close`, a variable with this name will be looked for in the userdata.

Example of use
##############

You can find a concrete example of how to use this state, :ref:`here <example_state_external_component>`.
