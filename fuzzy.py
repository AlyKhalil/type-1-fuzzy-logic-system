import numpy as np
import matplotlib.pyplot as plt


class Fuzzifier:
    """
    Class Responsible for fuzzifications of inputs
    NOTE: Each input should have its own Fuzzifier object
    """

    def __init__(self, fuzzy_set: list):
        self.fuzzy_set = fuzzy_set
        self.membership_values = None
        self.crisp_input = None

    def calc_membership_values(self, crisp_input: float):
        """
        Takes inputs read by the sensors and returns its membership values for each linguistic label

        Args:
            crisp_input: The sensor reading value

        Returns:
            Dictionary of membership values
        """
        membership_values = dict()
        self.crisp_input = crisp_input

        for ling_var, membership_fn in self.fuzzy_set:
            membership_values[ling_var] = membership_fn(crisp_input)

        self.membership_values = membership_values

        return membership_values

    def plot_membership_functions(
        self, x_range: tuple = (0, 1.5), num_points: int = 200
    ):
        """
        Plot all membership functions on one graph, with an optional crisp input marker
        """
        x = np.linspace(*x_range, num_points)
        plt.figure(figsize=(8, 5))

        # Plot each fuzzy membership function
        for ling_var, membership_fn in self.fuzzy_set:
            y = membership_fn(x)
            plt.plot(x, y, label=ling_var)

        # If a crisp input is provided, plot it as a vertical dashed line
        if self.crisp_input is not None:
            plt.axvline(
                x=self.crisp_input,
                color="red",
                linestyle="--",
                linewidth=2,
                label=f"Crisp Input = {self.crisp_input}",
            )

        plt.title("Fuzzy Membership Functions")
        plt.xlabel("Input (crisp value)")
        plt.ylabel("Membership degree")
        plt.legend()
        plt.grid(True)
        plt.show()


class Inference:
    """
    Performs fuzzy inference using the rule base and input Fuzzifier objects
    """

    def __init__(self, input_fuzzifiers: dict, rules: list):
        """
        Args:
            input_fuzzifiers: Dict mapping variable names to Fuzzifier objects
            rules: List of rules in format:
                   ((input1, label1), (input2, label2), ..., (output1, label1), (output2, label2), ...)
        """
        self.input_fuzzifiers = input_fuzzifiers
        self.rules = rules
        self.rule_strengths = []

    def compute_firing_strengths(self):
        """
        Calculates firing strengths for each rule using min-operator (Mamdani) and returns it.

        Returns:
            List: Containing ((rule), firing_strength)
        """
        self.rule_strengths = []
        for rule in self.rules:
            antecedent_strengths = []

            # Process antecedents (inputs)
            for component in rule:
                var_name, ling_label = component

                # Check if this is an input variable (has a fuzzifier)
                if var_name in self.input_fuzzifiers.keys():
                    fuzzifier = self.input_fuzzifiers[var_name]

                    if fuzzifier.membership_values is None:
                        raise ValueError(
                            f"Membership values not calculated for '{var_name}'"
                        )

                    mu = fuzzifier.membership_values.get(ling_label, 0)
                    antecedent_strengths.append(mu)
                else:
                    # If not, then this is an output/consequent, break
                    break

            # Calc AND (min) operator firing strength
            firing_strength = min(antecedent_strengths) if antecedent_strengths else 0
            self.rule_strengths.append((rule, firing_strength))

        return self.rule_strengths

    def print_rule_evaluation(self):
        """
        Pretty print the rule evaluations with firing strengths.

        Prints:
            IF ... AND ... THEN ...
        """
        print("\n" + "=" * 70)
        print("RULE EVALUATION")
        print("=" * 70)

        for idx, (rule, strength) in enumerate(self.rule_strengths, 1):
            # Separate antecedents and consequents
            antecedents = []
            consequents = []

            for component in rule:
                var_name, ling_label = component
                if var_name in self.input_fuzzifiers:
                    antecedents.append(f"{var_name}={ling_label}")
                else:
                    consequents.append(f"{var_name}={ling_label}")

            ant_str = " AND ".join(antecedents)
            cons_str = " AND ".join(consequents)

            print(f"Rule {idx}: IF {ant_str}")
            print(f"        THEN {cons_str}")
            print(f"        Firing Strength: {strength:.3f}")
            print("-" * 70)


class Defuzzifier:
    """
    Takes output peak values
    (Centre of sets of a symmetric output membership functions)
    and defuzzifies the output using the firing strengths and rule base
    """

    def __init__(self, rule_strengths: list, output_peaks: dict):
        self.rule_strengths = rule_strengths
        self.output_peaks = output_peaks

    def defuzzify(self):
        """
        Uses the rules' firing strengths and output peaks
        and calculates the final crisp outputs

        Returns:
            Dict(float): final crisp outputs
        """
        crisp_outputs = {}

        for ling_var, peak_values in self.output_peaks.items():
            numerator = 0.0
            denominator = 0.0

            for rule, strength in self.rule_strengths:
                # Finds the output label for this variable in the rule
                output_label = None
                for component in rule:
                    var_name, label = component
                    if var_name == ling_var:
                        output_label = label
                        break

                if output_label:
                    peak_value = peak_values[output_label]
                    numerator += peak_value * strength
                    denominator += strength

            crisp_outputs[ling_var] = float(
                numerator / denominator if denominator > 0.0 else 0.0
            )

        return crisp_outputs
