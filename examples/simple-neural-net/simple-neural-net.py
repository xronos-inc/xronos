# SPDX-FileCopyrightText: (c) 2024 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Any, Callable, List, Literal, TypedDict

import numpy as np
import xronos


class TrainedLayerDefinition(TypedDict):
    weights: np.ndarray
    biases: np.ndarray
    activation: Literal["relu", "sigmoid"]


class NeuralPredictor:
    def __init__(self, architecture: List[TrainedLayerDefinition]):
        """Initialize predictor with trained weights for any number of layers.

        This is produced in the save_architecture function
        """
        self.architecture = architecture

    def sigmoid(self, x: np.ndarray) -> Any:
        return 1 / (1 + np.exp(-np.clip(x, -100, 100)))

    def relu(self, x: np.ndarray) -> np.ndarray:
        return np.maximum(0, x)

    def get_activation(self, name: str) -> Callable[[np.ndarray], np.ndarray]:
        activations = {"relu": self.relu, "sigmoid": self.sigmoid}
        return activations[name]

    def predict(self, input_data: np.ndarray) -> float:
        # make one forward pass through each layer
        x = np.array(input_data).reshape(1, -1)
        for layer in self.architecture:
            z = np.dot(x, layer["weights"]) + layer["biases"]
            x = self.get_activation(layer["activation"])(z)

        return float(x[0, 0])


class TrainingController(xronos.Reactor):
    output_data = xronos.OutputPortDeclaration[float]()
    input_pred = xronos.InputPortDeclaration[float]()
    output_gradient = xronos.OutputPortDeclaration[float]()
    _training_timer = xronos.TimerDeclaration()

    def __init__(
        self,
        max_epochs: int = 1000,
        X: np.ndarray = np.array([]),
        y: np.ndarray = np.array([]),
    ):
        super().__init__()
        self.X = X
        self.y = y
        self.current_idx = 0
        self.epoch = 0
        self.max_epochs = max_epochs
        self._training_timer.period = datetime.timedelta(milliseconds=50)

    @xronos.reaction
    def on_timer(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self._training_timer)
        data_effect = interface.add_effect(self.output_data)

        def handler() -> None:
            if self.epoch >= self.max_epochs:
                self.environment.request_shutdown()
                return

            data_effect.set(self.X[self.current_idx])

        return handler

    @xronos.reaction
    def on_prediction(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        pred_trigger = interface.add_trigger(self.input_pred)
        gradient_effect = interface.add_effect(self.output_gradient)

        def handler() -> None:
            pred = pred_trigger.get()
            target = self.y[self.current_idx]

            pred = np.clip(pred, 1e-7, 1 - 1e-7)

            # choose your loss function, in this BCEL, becuase we need a 1 or a 0
            # we only actually use the derivative of this function for the gradient.
            # Uncomment and print if needed for debug purposes.

            # loss = -np.mean(target * np.log(pred) + (1 - target) * np.log(1 - pred))

            # take derivative of the loss function to calculate gradient
            d_pred = -((target / pred) - ((1 - target) / (1 - pred)))
            gradient_effect.set(d_pred)

            self.current_idx = (self.current_idx + 1) % len(self.X)
            if self.current_idx == 0:
                self.epoch += 1

        return handler


class DenseLayer(xronos.Reactor):
    input_data = xronos.InputPortDeclaration[float]()
    output_data = xronos.OutputPortDeclaration[float]()
    input_gradient = xronos.InputPortDeclaration[float]()
    output_gradient = xronos.OutputPortDeclaration[float]()

    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        learning_rate: float = 0.05,
        is_output: bool = False,
    ):
        super().__init__()
        self.weights = np.random.randn(input_dim, output_dim) * 0.1
        self.biases = np.zeros((1, output_dim))
        self.learning_rate = learning_rate
        self.last_input: np.ndarray | None = None
        self.last_z = None
        self.is_output = is_output

    def get_layer_data(self) -> TrainedLayerDefinition:
        return {
            "weights": self.weights.copy(),
            "biases": self.biases.copy(),
            "activation": "sigmoid" if self.is_output else "relu",
        }

    def relu(self, x: Any) -> Any:
        return np.maximum(0, x)

    def relu_derivative(self, x: Any) -> Any:
        return np.where(x > 0, 1, 0)

    def sigmoid(self, x: Any) -> Any:
        return 1 / (1 + np.exp(-np.clip(x, -100, 100)))

    def sigmoid_derivative(self, x: Any) -> Any:
        s = self.sigmoid(x)
        return s * (1 - s)

    @xronos.reaction
    def on_forward(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_trigger = interface.add_trigger(self.input_data)
        output_effect = interface.add_effect(self.output_data)

        def handler() -> None:
            input_data: np.ndarray = np.array(input_trigger.get()).reshape(1, -1)
            self.last_input = input_data

            z = np.dot(input_data, self.weights) + self.biases
            self.last_z = z

            if self.is_output:
                output_effect.set(self.sigmoid(z))
            else:
                output_effect.set(self.relu(z))

        return handler

    @xronos.reaction
    def on_backward(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        gradient_trigger = interface.add_trigger(self.input_gradient)
        gradient_effect = interface.add_effect(self.output_gradient)

        def handler() -> None:
            if self.last_input is None or self.last_z is None:
                return

            d_output = gradient_trigger.get()

            if self.is_output:
                d_activation = self.sigmoid_derivative(self.last_z)
            else:
                d_activation = self.relu_derivative(self.last_z)

            d_layer = d_output * d_activation

            d_weights = np.dot(self.last_input.T, d_layer)
            d_biases = np.sum(d_layer, axis=0, keepdims=True)

            self.weights -= self.learning_rate * d_weights
            self.biases -= self.learning_rate * d_biases

            gradient_effect.set(np.dot(d_layer, self.weights.T))

        return handler


"""These two functions save the weights so they can later be used as a model for
predictions.
"""


def save_architecture(
    layers: List[DenseLayer], filename: str = "network_weights.npz"
) -> None:
    save_dict = {}
    for i, layer in enumerate(layers):
        layer_data = layer.get_layer_data()
        save_dict[f"weights_{i}"] = layer_data["weights"]
        save_dict[f"biases_{i}"] = layer_data["biases"]
        save_dict[f"activation_{i}"] = layer_data["activation"]  # type: ignore

    np.savez(filename, **save_dict)  # type: ignore[arg-type]


def load_architecture(
    filename: str = "network_weights.npz",
) -> List[TrainedLayerDefinition]:
    with np.load(filename, allow_pickle=True) as data:
        # Count number of layers by looking at number of weight matrices
        num_layers = sum(1 for k in data.keys() if k.startswith("weights"))

        architecture = []
        for i in range(num_layers):
            layer_data: TrainedLayerDefinition = {
                "weights": data[f"weights_{i}"],
                "biases": data[f"biases_{i}"],
                "activation": str(data[f"activation_{i}"]),  # type: ignore
            }
            architecture.append(layer_data)

    return architecture


"""
Here we pass in training (X) and result (y) set
This example defaults to XOR for simplicity, but one
could pass arbitrary data.
"""


def train(
    layer_dims: List[int] = [2, 6, 1],
    epochs: int = 500,
    learning_rate: float = 0.06,
    X: np.ndarray = np.array([[0, 0], [0, 1], [1, 0], [1, 1]]),
    y: np.ndarray = np.array([[0], [1], [1], [0]]),
) -> NeuralPredictor:
    env = xronos.Environment()
    controller = env.create_reactor(
        "controller", TrainingController, max_epochs=epochs, X=X, y=y
    )

    layers: List[DenseLayer] = []

    for i in range(len(layer_dims) - 1):
        is_output = i == len(layer_dims) - 2
        layer = env.create_reactor(
            f"layer_{i}",
            DenseLayer,
            input_dim=layer_dims[i],
            output_dim=layer_dims[i + 1],
            learning_rate=learning_rate,
            is_output=is_output,
        )
        layers.append(layer)

    # connect the forward passes
    env.connect(controller.output_data, layers[0].input_data)
    for i in range(len(layers) - 1):
        env.connect(layers[i].output_data, layers[i + 1].input_data)
    env.connect(layers[-1].output_data, controller.input_pred)

    # connect the backward passes
    env.connect(controller.output_gradient, layers[-1].input_gradient)
    for i in range(len(layers) - 1, 0, -1):
        env.connect(layers[i].output_gradient, layers[i - 1].input_gradient)

    env.execute()

    save_architecture(layers)

    architecture = load_architecture()

    return NeuralPredictor(architecture)


if __name__ == "__main__":
    predictor = train(layer_dims=[2, 6, 1])

    test_inputs: List[List[int]] = [[0, 0], [0, 1], [1, 0], [1, 1]]
    print("Testing trained network:")
    for x in test_inputs:
        pred = predictor.predict(np.array(x))
        print(f"Input {x}: {pred}")
