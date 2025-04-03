// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

pub mod xronos {
    pub mod services {
        pub mod diagram_generator {
            tonic::include_proto!("xronos.services.diagram_generator");
            include!(concat!(
                env!("OUT_DIR"),
                "/xronos.services.diagram_generator.serde.rs"
            ));
        }
    }
    pub mod messages {
        pub mod reactor_graph {
            include!(concat!(
                env!("OUT_DIR"),
                "/xronos.messages.reactor_graph.rs"
            ));
            include!(concat!(
                env!("OUT_DIR"),
                "/xronos.messages.reactor_graph.serde.rs"
            ));
        }
        pub mod source_info {
            include!(concat!(
                env!("OUT_DIR"),
                "/xronos.messages.source_info.rs"
            ));
            include!(concat!(
                env!("OUT_DIR"),
                "/xronos.messages.source_info.serde.rs"
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::xronos::messages::reactor_graph::*;

    #[test]
    fn test_from_json() {
        let json = r#"
            {
             "elements": [
              {
               "name": "counter",
               "reactor": {
               }
              },
              {
               "name": "output",
               "uid": "4",
               "port": {
                "portType": "PORT_TYPE_OUTPUT"
               }
              },
              {
               "name": "startup",
               "uid": "1",
               "timer": {
                "period": "0s",
                "offset": "0s",
                "timerType": "TIMER_TYPE_STARTUP"
               }
              },
              {
               "name": "shutdown",
               "uid": "2",
               "timer": {
                "period": "0s",
                "offset": "0s",
                "timerType": "TIMER_TYPE_STARTUP"
               }
              },
              {
               "name": "timer",
               "uid": "3",
               "timer": {
                "period": "0.500s",
                "offset": "0s",
                "timerType": "TIMER_TYPE_GENERIC"
               }
              },
              {
               "name": "on_timer",
               "uid": "10",
               "reaction": {
                "priority": 0
               }
              },
              {
               "name": "printer",
               "uid": "5",
               "reactor": {
               }
              },
              {
               "name": "input",
               "uid": "8",
               "port": {
                "portType": "PORT_TYPE_INPUT"
               }
              },
              {
               "name": "startup",
               "uid": "6",
               "timer": {
                "period": "0s",
                "offset": "0s",
                "timerType": "TIMER_TYPE_STARTUP"
               }
              },
              {
               "name": "shutdown",
               "uid": "7",
               "timer": {
                "period": "0s",
                "offset": "0s",
                "timerType": "TIMER_TYPE_STARTUP"
               }
              },
              {
               "name": "on_input",
               "uid": "9",
               "reaction": {
                "priority": 1
               }
              }
             ],
             "connections": [
              {
               "fromUid": "4",
               "targets": [
                {
                 "toUid": "8",
                 "properties": {}
                }
               ]
              }
             ],
             "containments": [
              {
               "containeeUids": [
                "4",
                "1",
                "2",
                "3",
                "10"
               ]
              },
              {
               "containerUid": "5",
               "containeeUids": [
                "8",
                "6",
                "7",
                "9"
               ]
              }
             ],
             "dependencies": [
              {
               "reactionUid": "10",
               "triggerUids": [
                "3"
               ],
               "effectUids": [
                "4"
               ]
              },
              {
               "reactionUid": "9",
               "triggerUids": [
                "8"
               ]
              }
             ]
            }
        "#;
        let reactor_graph: Graph = serde_json::from_str(json).unwrap();
        let expected = expect_test::expect![[r#"
            Graph {
                elements: [
                    ReactorElement {
                        name: "counter",
                        uid: 0,
                        elem: Some(
                            Reactor(
                                ReactorInstance,
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "output",
                        uid: 4,
                        elem: Some(
                            Port(
                                Port {
                                    port_type: Output,
                                    data_type: None,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "startup",
                        uid: 1,
                        elem: Some(
                            Timer(
                                Timer {
                                    period: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    offset: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    timer_type: Startup,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "shutdown",
                        uid: 2,
                        elem: Some(
                            Timer(
                                Timer {
                                    period: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    offset: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    timer_type: Startup,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "timer",
                        uid: 3,
                        elem: Some(
                            Timer(
                                Timer {
                                    period: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 500000000,
                                        },
                                    ),
                                    offset: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    timer_type: Generic,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "on_timer",
                        uid: 10,
                        elem: Some(
                            Reaction(
                                Reaction {
                                    priority: 0,
                                    deadline: None,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "printer",
                        uid: 5,
                        elem: Some(
                            Reactor(
                                ReactorInstance,
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "input",
                        uid: 8,
                        elem: Some(
                            Port(
                                Port {
                                    port_type: Input,
                                    data_type: None,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "startup",
                        uid: 6,
                        elem: Some(
                            Timer(
                                Timer {
                                    period: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    offset: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    timer_type: Startup,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "shutdown",
                        uid: 7,
                        elem: Some(
                            Timer(
                                Timer {
                                    period: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    offset: Some(
                                        Duration {
                                            seconds: 0,
                                            nanos: 0,
                                        },
                                    ),
                                    timer_type: Startup,
                                },
                            ),
                        ),
                    },
                    ReactorElement {
                        name: "on_input",
                        uid: 9,
                        elem: Some(
                            Reaction(
                                Reaction {
                                    priority: 1,
                                    deadline: None,
                                },
                            ),
                        ),
                    },
                ],
                connections: [
                    Connection {
                        from_uid: 4,
                        targets: [
                            ConnectionTarget {
                                to_uid: 8,
                                properties: Some(
                                    ConnectionProperties {
                                        is_physical: false,
                                        delay: None,
                                    },
                                ),
                            },
                        ],
                    },
                ],
                containments: [
                    Containment {
                        container_uid: 0,
                        containee_uids: [
                            4,
                            1,
                            2,
                            3,
                            10,
                        ],
                    },
                    Containment {
                        container_uid: 5,
                        containee_uids: [
                            8,
                            6,
                            7,
                            9,
                        ],
                    },
                ],
                dependencies: [
                    ReactionDependencies {
                        reaction_uid: 10,
                        trigger_uids: [
                            3,
                        ],
                        source_uids: [],
                        effect_uids: [
                            4,
                        ],
                    },
                    ReactionDependencies {
                        reaction_uid: 9,
                        trigger_uids: [
                            8,
                        ],
                        source_uids: [],
                        effect_uids: [],
                    },
                ],
            }"#]];
        expected.assert_eq(&format!("{:#?}", reactor_graph));
    }
}
