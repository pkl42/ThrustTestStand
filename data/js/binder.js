export function bindElements(bindings) {
    bindings.forEach(({ id, selector, event = "click", handler }) => {
        const elements = selector
            ? document.querySelectorAll(selector)
            : [document.getElementById(id)];

        elements.forEach(el => {
            if (!el) return;

            el.addEventListener(event, (eventObj) => {
                handler({ el, event: eventObj });
            });
        });
    });
}